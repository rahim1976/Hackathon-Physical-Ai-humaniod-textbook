"""
Document Processor for the RAG Chatbot
This module handles the processing of various document formats for indexing.
"""

import os
import logging
from typing import List, Dict, Any
from pathlib import Path

import markdown
from bs4 import BeautifulSoup

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class DocumentProcessor:
    """
    Processes various document formats for the RAG system.
    Currently supports Markdown files, with extensibility for other formats.
    """

    def __init__(self):
        self.supported_extensions = {'.md', '.txt', '.html', '.htm'}

    def process_directory(self, directory_path: str) -> List[Dict[str, Any]]:
        """
        Process all supported documents in a directory.

        Args:
            directory_path: Path to the directory containing documents

        Returns:
            List of dictionaries containing document content and metadata
        """
        documents = []
        directory = Path(directory_path)

        for file_path in directory.rglob('*'):
            if file_path.suffix.lower() in self.supported_extensions:
                try:
                    content = self.process_file(str(file_path))
                    if content:
                        documents.append({
                            'source': str(file_path),
                            'content': content,
                            'metadata': {
                                'file_path': str(file_path),
                                'file_name': file_path.name,
                                'file_size': file_path.stat().st_size,
                                'extension': file_path.suffix.lower()
                            }
                        })
                        logger.info(f"Processed document: {file_path}")
                except Exception as e:
                    logger.error(f"Error processing file {file_path}: {str(e)}")

        return documents

    def process_file(self, file_path: str) -> str:
        """
        Process a single file based on its extension.

        Args:
            file_path: Path to the file to process

        Returns:
            Extracted text content from the file
        """
        with open(file_path, 'r', encoding='utf-8') as file:
            content = file.read()

        file_extension = Path(file_path).suffix.lower()

        if file_extension in ['.md', '.markdown']:
            return self._process_markdown(content)
        elif file_extension in ['.txt']:
            return content
        elif file_extension in ['.html', '.htm']:
            return self._process_html(content)
        else:
            # For unsupported formats, return as-is
            logger.warning(f"Unsupported file format: {file_extension}, treating as plain text")
            return content

    def _process_markdown(self, content: str) -> str:
        """
        Process Markdown content, converting to plain text while preserving structure.

        Args:
            content: Raw Markdown content

        Returns:
            Extracted text content
        """
        try:
            # Convert Markdown to HTML
            html = markdown.markdown(content)

            # Parse HTML and extract text
            soup = BeautifulSoup(html, 'html.parser')

            # Remove script and style elements
            for script in soup(["script", "style"]):
                script.decompose()

            # Get text and clean it up
            text = soup.get_text()

            # Clean up whitespace
            lines = (line.strip() for line in text.splitlines())
            chunks = (phrase.strip() for line in lines for phrase in line.split("  "))
            text = ' '.join(chunk for chunk in chunks if chunk)

            return text
        except Exception as e:
            logger.error(f"Error processing Markdown content: {str(e)}")
            return content  # Return original content if processing fails

    def _process_html(self, content: str) -> str:
        """
        Process HTML content, extracting text while preserving structure.

        Args:
            content: Raw HTML content

        Returns:
            Extracted text content
        """
        try:
            soup = BeautifulSoup(content, 'html.parser')

            # Remove script and style elements
            for script in soup(["script", "style"]):
                script.decompose()

            # Get text and clean it up
            text = soup.get_text()

            # Clean up whitespace
            lines = (line.strip() for line in text.splitlines())
            chunks = (phrase.strip() for line in lines for phrase in line.split("  "))
            text = ' '.join(chunk for chunk in chunks if chunk)

            return text
        except Exception as e:
            logger.error(f"Error processing HTML content: {str(e)}")
            return content  # Return original content if processing fails

    def process_content(self, content: str, source_type: str = 'markdown') -> str:
        """
        Process raw content based on its type.

        Args:
            content: Raw content string
            source_type: Type of content ('markdown', 'html', 'text')

        Returns:
            Extracted text content
        """
        if source_type == 'markdown':
            return self._process_markdown(content)
        elif source_type == 'html':
            return self._process_html(content)
        elif source_type == 'text':
            return content
        else:
            logger.warning(f"Unknown source type: {source_type}, treating as plain text")
            return content