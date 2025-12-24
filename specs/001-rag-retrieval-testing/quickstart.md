# Quickstart Guide: RAG Retrieval Enhancement

**Feature**: RAG Retrieval Enhancement
**Created**: 2025-12-21

## Overview

This guide helps you set up and use the enhanced RAG retrieval system with threshold filtering, standardized interface, and improved functionality to verify that stored vectors in Qdrant can be retrieved accurately.

## Prerequisites

- Python 3.11+
- Access to Qdrant vector database (with existing embeddings)
- Cohere API key
- Required Python packages (qdrant-client, cohere, python-dotenv)

## Setup

1. **Environment Configuration**
   ```bash
   # Ensure your .env file contains:
   QDRANT_URL="https://your-qdrant-url"
   QDRANT_API_KEY="your-qdrant-api-key"
   COHERE_API_KEY="your-cohere-api-key"
   ```

2. **Install Dependencies**
   ```bash
   cd backend/embedding_pipeline
   uv sync  # or pip install -r requirements.txt
   ```

## Usage

### Running the Retrieval Script

1. **Basic Retrieval Test**
   ```bash
   cd backend/embedding_pipeline
   python retrieving.py
   ```

2. **Retrieve All Stored Data**
   ```bash
   python retrieving.py all
   ```

3. **Golden Query Test**
   ```bash
   python retrieving.py test
   ```

4. **Stateless Behavior Test**
   ```bash
   python retrieving.py stateless
   ```

5. **Show Help**
   ```bash
   python retrieving.py --help
   ```

## Enhanced API Example

```python
from retrieving import RAGRetriever

# Initialize the retriever with custom timeouts (optional)
retriever = RAGRetriever(cohere_timeout=30, qdrant_timeout=30)

# Use the standardized retrieve_context function with threshold filtering
results = retriever.retrieve_context("your search query", k=5, threshold=0.7)

# Or use the original retrieve method
json_response = retriever.retrieve("your search query", top_k=5, threshold=0.0)
```

## New Functionality

### Standardized retrieve_context Function
The system provides a standardized function for retrieval with threshold filtering:

```python
results = retriever.retrieve_context("Your query here", k=5, threshold=0.7)
```

### Threshold Filtering
Results are automatically filtered to include only those with similarity scores above the threshold (default: 0.7).

### Timeout Handling
The system includes timeout handling for both Cohere and Qdrant API calls:
- Cohere API: 30 second timeout (configurable)
- Qdrant API: 30 second timeout (configurable)

### Enhanced Metadata
Each result includes comprehensive metadata:
- content: The retrieved text content
- url: Source URL
- position: Position in the original document
- similarity_score: Semantic similarity score
- chunk_id: Unique identifier
- created_at: Creation timestamp
- section: Section information (if available)
- heading: Heading information (if available)

## Expected Output

The retrieval script will output JSON with the following structure:

```json
{
  "query": "your search query",
  "results": [
    {
      "content": "retrieved text content",
      "url": "source-url",
      "position": 0,
      "similarity_score": 0.95,
      "chunk_id": "unique-id",
      "created_at": 1234567890,
      "section": "section name",
      "heading": "heading name"
    }
  ],
  "metadata": {
    "query_time_ms": 1234,
    "total_results": 5,
    "timestamp": 1234567890,
    "collection_name": "rag_embedding"
  }
}
```

## Testing Capabilities

The system includes comprehensive testing capabilities:
- Golden query validation for expected book metadata
- Stateless behavior verification
- Threshold filtering validation
- Performance optimization testing
- Timeout handling validation