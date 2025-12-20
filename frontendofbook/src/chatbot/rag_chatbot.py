"""
RAG (Retrieval-Augmented Generation) Chatbot Implementation
This module implements the core functionality for an AI chatbot that grounds responses
in documented content to prevent hallucinations.
"""

import os
import logging
from typing import List, Dict, Any, Optional
from dataclasses import dataclass

import openai
from openai import OpenAI
from qdrant_client import QdrantClient
from qdrant_client.http import models
import tiktoken

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

@dataclass
class DocumentChunk:
    """Represents a chunk of a document with metadata."""
    id: str
    content: str
    source: str
    metadata: Dict[str, Any]
    embedding: Optional[List[float]] = None

class RAGChatbot:
    """
    A Retrieval-Augmented Generation chatbot that ensures responses
    are grounded in documented content to prevent hallucinations.
    """

    def __init__(self,
                 openai_api_key: str,
                 qdrant_url: str,
                 qdrant_api_key: str,
                 collection_name: str = "document_chunks",
                 model: str = "gpt-4-turbo",
                 embedding_model: str = "text-embedding-3-small"):

        # Initialize OpenAI client
        self.openai_client = OpenAI(api_key=openai_api_key)
        self.model = model
        self.embedding_model = embedding_model

        # Initialize Qdrant client
        self.qdrant_client = QdrantClient(
            url=qdrant_url,
            api_key=qdrant_api_key,
            prefer_grpc=True
        )
        self.collection_name = collection_name

        # Initialize tokenizer for chunking
        self.tokenizer = tiktoken.encoding_for_model(self.model)

        # Create collection if it doesn't exist
        self._create_collection()

        logger.info("RAG Chatbot initialized successfully")

    def _create_collection(self):
        """Create the Qdrant collection for storing document embeddings."""
        try:
            # Check if collection exists
            self.qdrant_client.get_collection(self.collection_name)
            logger.info(f"Collection '{self.collection_name}' already exists")
        except:
            # Create new collection
            self.qdrant_client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(
                    size=1536,  # Standard size for text-embedding-3-small
                    distance=models.Distance.COSINE
                )
            )
            logger.info(f"Created collection '{self.collection_name}'")

    def chunk_document(self,
                      content: str,
                      source: str,
                      max_chunk_size: int = 1000,
                      overlap: int = 100) -> List[DocumentChunk]:
        """
        Split a document into semantic chunks.

        Args:
            content: The document content to chunk
            source: Source identifier for the document
            max_chunk_size: Maximum number of tokens per chunk
            overlap: Number of tokens to overlap between chunks

        Returns:
            List of DocumentChunk objects
        """
        tokens = self.tokenizer.encode(content)
        chunks = []
        start_idx = 0

        while start_idx < len(tokens):
            # Determine the end index for this chunk
            end_idx = min(start_idx + max_chunk_size, len(tokens))

            # Decode the token slice back to text
            chunk_tokens = tokens[start_idx:end_idx]
            chunk_text = self.tokenizer.decode(chunk_tokens)

            # Create chunk ID
            chunk_id = f"{source}_chunk_{len(chunks)}"

            # Create document chunk
            chunk = DocumentChunk(
                id=chunk_id,
                content=chunk_text,
                source=source,
                metadata={
                    "start_idx": start_idx,
                    "end_idx": end_idx,
                    "source": source
                }
            )

            chunks.append(chunk)

            # Move start index forward, accounting for overlap
            start_idx = end_idx - overlap if end_idx < len(tokens) else end_idx

        return chunks

    def embed_text(self, text: str) -> List[float]:
        """
        Generate embedding for a text string.

        Args:
            text: The text to embed

        Returns:
            List of embedding values
        """
        response = self.openai_client.embeddings.create(
            input=text,
            model=self.embedding_model
        )
        return response.data[0].embedding

    def embed_chunks(self, chunks: List[DocumentChunk]) -> List[DocumentChunk]:
        """
        Generate embeddings for a list of document chunks.

        Args:
            chunks: List of DocumentChunk objects without embeddings

        Returns:
            List of DocumentChunk objects with embeddings
        """
        # Extract text content for batch embedding
        texts = [chunk.content for chunk in chunks]

        # Generate embeddings in batch
        response = self.openai_client.embeddings.create(
            input=texts,
            model=self.embedding_model
        )

        # Assign embeddings back to chunks
        for i, chunk in enumerate(chunks):
            chunk.embedding = response.data[i].embedding

        return chunks

    def index_document(self, content: str, source: str) -> bool:
        """
        Index a document by chunking it, generating embeddings, and storing in Qdrant.

        Args:
            content: The document content to index
            source: Source identifier for the document

        Returns:
            True if indexing was successful
        """
        try:
            # Chunk the document
            chunks = self.chunk_document(content, source)
            logger.info(f"Chunked document into {len(chunks)} chunks")

            # Generate embeddings for chunks
            chunks_with_embeddings = self.embed_chunks(chunks)
            logger.info(f"Generated embeddings for {len(chunks_with_embeddings)} chunks")

            # Prepare points for Qdrant
            points = []
            for chunk in chunks_with_embeddings:
                points.append(models.PointStruct(
                    id=chunk.id,
                    vector=chunk.embedding,
                    payload={
                        "content": chunk.content,
                        "source": chunk.source,
                        "metadata": chunk.metadata
                    }
                ))

            # Upload points to Qdrant
            self.qdrant_client.upsert(
                collection_name=self.collection_name,
                points=points
            )

            logger.info(f"Successfully indexed {len(points)} chunks to Qdrant")
            return True

        except Exception as e:
            logger.error(f"Error indexing document: {str(e)}")
            return False

    def retrieve_relevant_chunks(self,
                                query: str,
                                top_k: int = 5,
                                threshold: float = 0.5) -> List[Dict[str, Any]]:
        """
        Retrieve the most relevant document chunks for a query.

        Args:
            query: The user's query
            top_k: Number of top results to return
            threshold: Minimum similarity score threshold

        Returns:
            List of relevant chunks with their metadata
        """
        try:
            # Generate embedding for the query
            query_embedding = self.embed_text(query)

            # Search in Qdrant
            search_results = self.qdrant_client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=top_k,
                score_threshold=threshold
            )

            # Extract relevant chunks
            relevant_chunks = []
            for result in search_results:
                relevant_chunks.append({
                    "id": result.id,
                    "content": result.payload["content"],
                    "source": result.payload["source"],
                    "metadata": result.payload["metadata"],
                    "similarity_score": result.score
                })

            logger.info(f"Retrieved {len(relevant_chunks)} relevant chunks for query")
            return relevant_chunks

        except Exception as e:
            logger.error(f"Error retrieving relevant chunks: {str(e)}")
            return []

    def generate_response(self,
                         query: str,
                         relevant_chunks: List[Dict[str, Any]],
                         max_tokens: int = 1000,
                         temperature: float = 0.7) -> str:
        """
        Generate a response based on the query and relevant document chunks.

        Args:
            query: The user's query
            relevant_chunks: List of relevant document chunks
            max_tokens: Maximum number of tokens in the response
            temperature: Temperature parameter for response generation

        Returns:
            Generated response string
        """
        try:
            # Construct context from relevant chunks
            context_parts = ["Based on the provided documentation:"]
            for i, chunk in enumerate(relevant_chunks):
                context_parts.append(f"\nDocument {i+1} ({chunk['source']}):")
                context_parts.append(f"{chunk['content']}")

            context = "\n".join(context_parts)

            # Create the prompt for the language model
            prompt = f"""
            You are an AI assistant for a documentation system. Your responses must be based solely on the provided documentation context. Do not generate information that is not contained in the context.

            Documentation Context:
            {context}

            User Query:
            {query}

            Please provide a helpful response based on the documentation context. If the documentation does not contain information to answer the query, please state that the information is not available in the provided documentation.
            """

            # Generate response using OpenAI
            response = self.openai_client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": "You are a helpful assistant that answers questions based only on provided documentation. Do not hallucinate information."},
                    {"role": "user", "content": prompt}
                ],
                max_tokens=max_tokens,
                temperature=temperature
            )

            return response.choices[0].message.content.strip()

        except Exception as e:
            logger.error(f"Error generating response: {str(e)}")
            return "I encountered an error while generating a response. Please try again."

    def chat(self,
             query: str,
             top_k: int = 5,
             threshold: float = 0.5,
             max_tokens: int = 1000,
             temperature: float = 0.7) -> Dict[str, Any]:
        """
        Main chat method that orchestrates the RAG process.

        Args:
            query: The user's query
            top_k: Number of relevant chunks to retrieve
            threshold: Minimum similarity threshold
            max_tokens: Maximum tokens in response
            temperature: Response temperature

        Returns:
            Dictionary containing response and metadata
        """
        try:
            # Retrieve relevant chunks
            relevant_chunks = self.retrieve_relevant_chunks(query, top_k, threshold)

            if not relevant_chunks:
                return {
                    "response": "I couldn't find relevant information in the documentation to answer your query.",
                    "sources": [],
                    "chunks_used": 0
                }

            # Generate response
            response_text = self.generate_response(
                query,
                relevant_chunks,
                max_tokens,
                temperature
            )

            # Extract source information
            sources = list(set([chunk["source"] for chunk in relevant_chunks]))

            return {
                "response": response_text,
                "sources": sources,
                "chunks_used": len(relevant_chunks),
                "relevant_chunks": relevant_chunks
            }

        except Exception as e:
            logger.error(f"Error in chat method: {str(e)}")
            return {
                "response": "I encountered an error processing your query. Please try again.",
                "sources": [],
                "chunks_used": 0
            }