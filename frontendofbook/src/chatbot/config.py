"""
Configuration utilities for the RAG Chatbot
Handles environment variables and application settings.
"""

import os
from typing import Optional
from dataclasses import dataclass

@dataclass
class ChatbotConfig:
    """Configuration class for the RAG Chatbot."""

    # OpenAI Configuration
    openai_api_key: str
    chatbot_model: str = "gpt-4-turbo"
    embedding_model: str = "text-embedding-3-small"

    # Qdrant Configuration
    qdrant_url: str
    qdrant_api_key: str
    collection_name: str = "document_chunks"

    # Document Processing Configuration
    book_content_path: str = "./docs"
    max_chunk_size: int = 1000
    chunk_overlap: int = 100

    # RAG Configuration
    retrieval_top_k: int = 5
    retrieval_threshold: float = 0.5
    max_tokens: int = 1000
    temperature: float = 0.7

    # Database Configuration
    database_url: Optional[str] = None

def load_config_from_env() -> ChatbotConfig:
    """
    Load configuration from environment variables.

    Returns:
        ChatbotConfig instance with values from environment variables

    Raises:
        ValueError: If required environment variables are missing
    """
    # Required environment variables
    required_vars = [
        'OPENAI_API_KEY',
        'QDRANT_URL',
        'QDRANT_API_KEY'
    ]

    missing_vars = [var for var in required_vars if not os.getenv(var)]
    if missing_vars:
        raise ValueError(f"Missing required environment variables: {', '.join(missing_vars)}")

    # Create config with environment values and defaults
    config = ChatbotConfig(
        openai_api_key=os.getenv('OPENAI_API_KEY'),
        qdrant_url=os.getenv('QDRANT_URL'),
        qdrant_api_key=os.getenv('QDRANT_API_KEY'),
        chatbot_model=os.getenv('CHATBOT_MODEL', 'gpt-4-turbo'),
        embedding_model=os.getenv('EMBEDDING_MODEL', 'text-embedding-3-small'),
        collection_name=os.getenv('QDRANT_COLLECTION_NAME', 'document_chunks'),
        book_content_path=os.getenv('BOOK_CONTENT_PATH', './docs'),
        max_chunk_size=int(os.getenv('MAX_CHUNK_SIZE', '1000')),
        chunk_overlap=int(os.getenv('CHUNK_OVERLAP', '100')),
        retrieval_top_k=int(os.getenv('RETRIEVAL_TOP_K', '5')),
        retrieval_threshold=float(os.getenv('RETRIEVAL_THRESHOLD', '0.5')),
        max_tokens=int(os.getenv('MAX_TOKENS', '1000')),
        temperature=float(os.getenv('TEMPERATURE', '0.7')),
        database_url=os.getenv('DATABASE_URL')
    )

    return config

def validate_config(config: ChatbotConfig) -> bool:
    """
    Validate the configuration values.

    Args:
        config: ChatbotConfig instance to validate

    Returns:
        True if configuration is valid, False otherwise
    """
    # Validate API keys are not empty
    if not config.openai_api_key or not config.qdrant_api_key:
        return False

    # Validate URLs are not empty
    if not config.qdrant_url:
        return False

    # Validate numeric values are positive
    if (config.max_chunk_size <= 0 or
        config.chunk_overlap < 0 or
        config.retrieval_top_k <= 0 or
        config.retrieval_threshold < 0 or
        config.max_tokens <= 0 or
        config.temperature < 0):
        return False

    return True