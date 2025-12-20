"""
Main application file for the RAG Chatbot
This file orchestrates the entire RAG system from document indexing to response generation.
"""

import os
import logging
from typing import List, Dict, Any

from .rag_chatbot import RAGChatbot
from .document_processor import DocumentProcessor
from .config import load_config_from_env, validate_config, ChatbotConfig
from .validation import ResponseValidator

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class ChatbotApplication:
    """
    Main application class that manages the RAG chatbot lifecycle.
    """

    def __init__(self, config: ChatbotConfig):
        self.config = config

        # Validate configuration
        if not validate_config(config):
            raise ValueError("Invalid configuration provided")

        # Initialize components
        self.document_processor = DocumentProcessor()
        self.chatbot = RAGChatbot(
            openai_api_key=config.openai_api_key,
            qdrant_url=config.qdrant_url,
            qdrant_api_key=config.qdrant_api_key,
            collection_name=config.collection_name,
            model=config.chatbot_model,
            embedding_model=config.embedding_model
        )
        self.validator = ResponseValidator(threshold=0.7)

        logger.info("Chatbot application initialized successfully")

    def index_documentation(self, content_path: str = None) -> bool:
        """
        Index all documentation in the specified path.

        Args:
            content_path: Path to documentation (defaults to config value)

        Returns:
            True if indexing was successful
        """
        if content_path is None:
            content_path = self.config.book_content_path

        logger.info(f"Starting documentation indexing from: {content_path}")

        try:
            # Process all documents in the directory
            documents = self.document_processor.process_directory(content_path)
            logger.info(f"Found {len(documents)} documents to index")

            # Index each document
            success_count = 0
            for doc in documents:
                success = self.chatbot.index_document(
                    content=doc['content'],
                    source=doc['source']
                )
                if success:
                    success_count += 1

            logger.info(f"Successfully indexed {success_count}/{len(documents)} documents")
            return success_count == len(documents)

        except Exception as e:
            logger.error(f"Error during documentation indexing: {str(e)}")
            return False

    def chat(self, query: str) -> Dict[str, Any]:
        """
        Process a chat query through the RAG system.

        Args:
            query: The user's query

        Returns:
            Dictionary containing the response and metadata
        """
        # Get the initial response from the RAG system
        result = self.chatbot.chat(
            query=query,
            top_k=self.config.retrieval_top_k,
            threshold=self.config.retrieval_threshold,
            max_tokens=self.config.max_tokens,
            temperature=self.config.temperature
        )

        # Perform comprehensive validation
        validation_result = self.validator.comprehensive_validation(
            query=query,
            response=result["response"],
            retrieved_chunks=result.get("relevant_chunks", [])
        )

        # Add validation information to the result
        result["validation"] = validation_result

        # Log validation results
        if not validation_result["is_valid"]:
            logger.warning(f"Response validation failed for query: {query[:100]}...")
            logger.warning(f"Validation issues: {validation_result['issues']}")

        return result

    def validate_response_grounding(self, query: str, response: str) -> bool:
        """
        Validate that a response is properly grounded in the documentation.

        Args:
            query: The original query
            response: The generated response

        Returns:
            True if response is properly grounded, False otherwise
        """
        # This is a basic validation - in a production system, you'd want more sophisticated checks
        # For now, we just ensure the response isn't a generic fallback
        if "not available in the provided documentation" in response.lower():
            return True  # This is an acceptable response when info isn't available

        # Check if response contains references to documentation
        # In a more sophisticated system, we'd check that claims in the response
        # are supported by the retrieved chunks
        return len(response.strip()) > 0

def main():
    """
    Main entry point for the application.
    """
    try:
        # Load configuration from environment
        config = load_config_from_env()
        logger.info("Configuration loaded successfully")

        # Create application instance
        app = ChatbotApplication(config)

        # Index documentation if specified
        if os.getenv('INDEX_DOCUMENTATION', 'true').lower() == 'true':
            success = app.index_documentation()
            if not success:
                logger.error("Documentation indexing failed")
                return 1

        logger.info("Application started successfully")

        # Example usage (in a real application, this would be handled by the web interface)
        print("RAG Chatbot is ready!")
        print("Type 'quit' to exit")

        while True:
            query = input("\nYour question: ")
            if query.lower() in ['quit', 'exit', 'q']:
                break

            response_data = app.chat(query)
            print(f"\nResponse: {response_data['response']}")

            if response_data['sources']:
                print(f"Sources: {', '.join(response_data['sources'])}")

        return 0

    except Exception as e:
        logger.error(f"Application error: {str(e)}")
        return 1

if __name__ == "__main__":
    exit(main())