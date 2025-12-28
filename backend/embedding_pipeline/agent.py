"""
OpenRouter Agent with Retrieval Integration

This module implements a retrieval-enabled agent using the OpenAI-compatible API
that integrates with OpenRouter API and uses the existing retrieval functionality
from retrieving.py.
"""
import os
import logging
from typing import Dict, List, Any, Optional
import json

from openai import OpenAI
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Import the retrieval functionality from the existing file
from retrieving import RAGRetriever

class OpenRouterAgentRAG:
    """
    A retrieval-enabled agent using OpenAI-compatible API that integrates with OpenRouter.
    Uses manual integration with the retrieval system.
    """

    def __init__(self,
                 openrouter_api_key: Optional[str] = None,
                 qdrant_url: Optional[str] = None,
                 qdrant_api_key: Optional[str] = None,
                 agent_instructions: Optional[str] = None):

        # Get API keys from parameters or environment
        self.openrouter_api_key = openrouter_api_key or os.getenv("OPENROUTER_API_KEY")
        self.qdrant_url = qdrant_url or os.getenv("QDRANT_URL", "http://localhost:6333")
        self.qdrant_api_key = qdrant_api_key or os.getenv("QDRANT_API_KEY")

        if not self.openrouter_api_key:
            raise ValueError("OPENROUTER_API_KEY is required")

        # Initialize OpenAI client with OpenRouter base URL
        self.openrouter_client = OpenAI(
            api_key=self.openrouter_api_key,
            base_url="https://openrouter.ai/api/v1"
        )

        # Set up default agent instructions if not provided
        if agent_instructions is None:
            agent_instructions = """
            Role: You are a professional Technical Assistant for the "Physical AI & Humanoid Robotics" textbook.

            Interaction Protocol:
 
            Greetings: If the user says "Hi," "Hello," or asks general pleasantries, respond naturally and politely without using any tools.

            Technical Queries (Priority): For any question regarding robotics, ROS 2, Gazebo, Unity, or physical AI, you MUST call the Qdrant retrieval tool first.

            Search Logic: Treat keywords like "ROS 2," "Humanoid," "Locomotion," and "Sensors" as high-priority triggers for documentation retrieval.

            Handling Results: > * If the tool returns data, synthesize a clear answer and always cite the source provided in the retrieved chunk.

            Only if the tool explicitly returns no relevant results after a thorough search should you state that the information is missing from the documentation.

            Constraint: Do not assume information is missing until the Qdrant retrieval tool has been executed and confirmed empty.
            """
        self.agent_instructions = agent_instructions

        logger.info("OpenRouter Agent with retrieval integration initialized successfully")

    def retrieve_content(self, query: str, top_k: int = 5, threshold: float = 0.5) -> List[Dict[str, Any]]:
        """
        Retrieve content using the existing retrieval system.
        """
        try:
            # Initialize the RAGRetriever from the existing module
            retriever = RAGRetriever()

            # Use the existing retrieve_context function
            results = retriever.retrieve_context(
                query=query,
                k=top_k,
                threshold=threshold
            )

            logger.info(f"Retrieved {len(results)} relevant chunks for query: {query}")
            return results

        except Exception as e:
            logger.error(f"Error retrieving content from Qdrant: {str(e)}")
            return []

    def chat(self, user_query: str, top_k: int = 5, threshold: float = 0.5) -> Dict[str, Any]:
        """
        Main chat method that uses the OpenRouter API to answer user queries.

        Args:
            user_query: The user's question
            top_k: Number of relevant chunks to retrieve
            threshold: Minimum similarity threshold

        Returns:
            Dictionary containing the agent's response and metadata
        """
        try:
            # First, retrieve relevant content using our retrieval system
            retrieved_chunks = self.retrieve_content(user_query, top_k, threshold)

            if retrieved_chunks:
                # Format the retrieved content as context for the LLM
                context_parts = ["Based on the provided documentation:"]
                for i, chunk in enumerate(retrieved_chunks, 1):
                    context_parts.append(f"\nDocument {i} (URL: {chunk.get('url', 'N/A')}):")
                    context_parts.append(f"Content: {chunk.get('content', '')}")
                    if 'section' in chunk and chunk.get('section'):
                        context_parts.append(f"Section: {chunk['section']}")
                    if 'heading' in chunk and chunk.get('heading'):
                        context_parts.append(f"Heading: {chunk['heading']}")
                    context_parts.append("---")

                context = "\n".join(context_parts)

                # Create the prompt for the language model
                prompt = f"""
                You are a helpful assistant that answers questions based only on provided documentation. Do not hallucinate information.

                Documentation Context:
                {context}

                User Query:
                {user_query}

                Please provide a helpful response based on the documentation context. Include relevant source information (URLs) in your response when possible. If the documentation does not contain information to answer the query, please state that the information is not available in the provided documentation.
                """
            else:
                # No relevant content found, inform the model
                prompt = f"""
                You are a helpful assistant. The user asked: "{user_query}"

                However, no relevant information was found in the documentation to answer this query. Please inform the user that the information is not available in the provided documentation.
                """

            # Call the OpenRouter API using chat completions
            response = self.openrouter_client.chat.completions.create(
                model="tngtech/deepseek-r1t2-chimera:free",  # Using the specified free model
                messages=[
                    {
                        "role": "system",
                        "content": self.agent_instructions
                    },
                    {
                        "role": "user",
                        "content": prompt
                    }
                ],
                temperature=0.7
            )

            # Extract the response text
            response_text = response.choices[0].message.content

            # Extract source information from retrieved chunks
            sources = list(set([chunk["url"] for chunk in retrieved_chunks if chunk["url"]]))

            result = {
                "response": response_text,
                "sources": sources,
                "chunks_used": len(retrieved_chunks),
                "relevant_chunks": retrieved_chunks,
                "status": "success"
            }

            logger.info(f"Agent successfully processed query: {user_query[:50]}...")
            return result

        except Exception as e:
            logger.error(f"Error in agent chat: {str(e)}")
            return {
                "response": "I encountered an error processing your query. Please try again.",
                "sources": [],
                "chunks_used": 0,
                "status": "error",
                "error": str(e)
            }

    def __del__(self):
        """
        Cleanup method to delete the assistant when the object is destroyed.
        """
        pass  # No cleanup needed for this version


def main():
    """
    Main function to demonstrate the OpenRouter Agent with retrieval integration.
    """
    logger.info("Initializing OpenRouter Agent with retrieval integration...")

    # Initialize the agent
    agent = OpenRouterAgentRAG()

    print("OpenRouter Agent with Retrieval Integration - Demo")
    print("=" * 60)
    print("Ask questions about robotics, ROS2, humanoid design, etc.")
    print("Type 'quit' to exit")
    print()

    while True:
        user_input = input("Your question: ").strip()

        if user_input.lower() in ['quit', 'exit', 'q']:
            print("Goodbye!")
            break

        if not user_input:
            continue

        print("\nProcessing your query...")
        result = agent.chat(user_input)

        print(f"\nResponse: {result['response']}")

        if result['sources']:
            print(f"\nSources: {len(result['sources'])} documents referenced")
            for i, source in enumerate(result['sources'][:3], 1):  # Show first 3 sources
                print(f"  {i}. {source}")

        print("-" * 60)


if __name__ == "__main__":
    main()