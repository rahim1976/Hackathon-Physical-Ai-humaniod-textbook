"""
End-to-end tests for the AI/Spec-Driven Book with Embedded RAG Chatbot
"""

import os
import pytest
import asyncio
from typing import Dict, Any

from src.chatbot.main import ChatbotApplication
from src.chatbot.config import ChatbotConfig
from src.api.chatbot_api import ChatRequest


class TestEndToEndFunctionality:
    """Test suite for end-to-end functionality of the RAG chatbot system."""

    @pytest.fixture(scope="class")
    def chatbot_app(self):
        """Create a chatbot application instance for testing."""
        # Use test configuration
        config = ChatbotConfig(
            openai_api_key=os.getenv('OPENAI_API_KEY', 'test-key'),
            qdrant_url=os.getenv('QDRANT_URL', 'http://localhost:6333'),
            qdrant_api_key=os.getenv('QDRANT_API_KEY', 'test-key'),
            collection_name='test_document_chunks',
            max_chunk_size=500,
            retrieval_top_k=3
        )

        # Skip tests if required environment variables are not set
        if config.openai_api_key == 'test-key':
            pytest.skip("OPENAI_API_KEY not set, skipping integration tests")

        app = ChatbotApplication(config)
        yield app

    def test_document_indexing(self, chatbot_app):
        """Test that documents can be properly indexed."""
        # Sample documentation content
        sample_content = """
        # Getting Started with AI Integration

        This guide explains how to integrate AI features into your application.

        ## Key Concepts

        - Machine Learning: A subset of AI that enables systems to learn and improve
        - Natural Language Processing: Technology that helps computers understand human language
        - Retrieval-Augmented Generation: Combines information retrieval with text generation

        ## Implementation Steps

        1. Set up your development environment
        2. Configure API keys and endpoints
        3. Implement the core functionality
        4. Test and validate the system
        """

        # Index the sample content
        success = chatbot_app.chatbot.index_document(
            content=sample_content,
            source="test_document.md"
        )

        assert success, "Document indexing should succeed"

    def test_chat_response_generation(self, chatbot_app):
        """Test that the chatbot generates appropriate responses."""
        # First, ensure we have some content indexed
        sample_content = """
        # AI Documentation
        The RAG (Retrieval-Augmented Generation) system combines information retrieval with text generation.
        It works by first retrieving relevant documents and then generating responses based on those documents.
        This approach helps ensure that responses are grounded in actual documentation rather than hallucinated.
        """

        chatbot_app.chatbot.index_document(
            content=sample_content,
            source="test_doc.md"
        )

        # Test a query that should be answerable from the indexed content
        result = chatbot_app.chat("What is the RAG system?")

        assert "response" in result
        assert len(result["response"]) > 0
        assert result["chunks_used"] > 0
        assert "RAG" in result["response"] or "retrieval" in result["response"].lower()

    def test_response_validation(self, chatbot_app):
        """Test that responses are properly validated for hallucination."""
        # Index some content
        sample_content = """
        # System Requirements
        The minimum system requirements are:
        - Operating System: Windows 10 or later, macOS 10.15 or later, or Linux
        - RAM: 8 GB or more
        - Storage: 500 MB available space
        - Internet connection: Required for activation
        """

        chatbot_app.chatbot.index_document(
            content=sample_content,
            source="requirements.md"
        )

        # Test a query about system requirements
        result = chatbot_app.chat("What are the system requirements?")

        # Check that the response exists and includes validation information
        assert "response" in result
        assert "validation" in result
        assert isinstance(result["validation"], dict)
        assert "is_valid" in result["validation"]

        # The response should be valid (not hallucinated)
        assert result["validation"]["is_valid"] is True

    def test_unanswerable_query(self, chatbot_app):
        """Test how the system handles queries that cannot be answered from documentation."""
        # Index some content
        sample_content = """
        # User Guide
        This guide covers basic usage of the application.
        For advanced features, see the Advanced Guide.
        """

        chatbot_app.chatbot.index_document(
            content=sample_content,
            source="user_guide.md"
        )

        # Test a query about something not in the documentation
        result = chatbot_app.chat("What is the meaning of life?")

        # The response should indicate that the information is not in the documentation
        assert "response" in result
        response_lower = result["response"].lower()
        assert ("not available" in response_lower or
                "not found" in response_lower or
                "documentation" in response_lower or
                "not contain" in response_lower)

    def test_conversation_context(self, chatbot_app):
        """Test that the system maintains conversation context appropriately."""
        # Index content about a specific topic
        sample_content = """
        # Product Features
        Our product includes several key features:
        - Feature A: Provides enhanced performance
        - Feature B: Offers improved security
        - Feature C: Delivers better user experience
        """

        chatbot_app.chatbot.index_document(
            content=sample_content,
            source="features.md"
        )

        # First query about features
        result1 = chatbot_app.chat("What features does the product have?")

        # Follow-up query that should reference previous context
        result2 = chatbot_app.chat("Tell me more about Feature A")

        assert "response" in result1
        assert "response" in result2
        assert len(result1["response"]) > 0
        assert len(result2["response"]) > 0


class TestAPIEndpoints:
    """Test the API endpoints for the RAG chatbot."""

    def test_health_check(self):
        """Test the health check endpoint."""
        # This would typically be tested via HTTP request
        # For now, we'll just verify the concept
        assert True  # Placeholder - actual implementation would test the API endpoint

    def test_config_endpoint(self):
        """Test the configuration endpoint."""
        # This would typically be tested via HTTP request
        assert True  # Placeholder - actual implementation would test the API endpoint


class TestDocumentProcessing:
    """Test document processing functionality."""

    def test_markdown_processing(self):
        """Test processing of Markdown documents."""
        from src.chatbot.document_processor import DocumentProcessor

        processor = DocumentProcessor()

        markdown_content = """
        # Title
        This is a sample document.

        ## Section
        Some content here.

        - List item 1
        - List item 2
        """

        processed = processor._process_markdown(markdown_content)

        # Should extract text content without markdown formatting
        assert "Title" in processed
        assert "Section" in processed
        assert "List item 1" in processed
        assert "#" not in processed  # Markdown symbols should be removed

    def test_html_processing(self):
        """Test processing of HTML documents."""
        from src.chatbot.document_processor import DocumentProcessor

        processor = DocumentProcessor()

        html_content = """
        <html>
        <head><title>Test Document</title></head>
        <body>
        <h1>Main Title</h1>
        <p>This is a sample document with <em>emphasized</em> text.</p>
        <ul>
        <li>List item 1</li>
        <li>List item 2</li>
        </ul>
        </body>
        </html>
        """

        processed = processor._process_html(html_content)

        # Should extract text content without HTML tags
        assert "Main Title" in processed
        assert "emphasized" in processed
        assert "List item 1" in processed
        assert "<h1>" not in processed  # HTML tags should be removed


if __name__ == "__main__":
    pytest.main([__file__])