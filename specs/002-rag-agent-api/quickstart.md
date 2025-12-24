# Quickstart Guide: OpenAI Agent with Qdrant Integration

**Feature**: OpenAI Agent with Qdrant Integration
**Created**: 2025-12-21

## Overview

This guide helps you set up and use the retrieval-enabled agent using OpenAI Agents SDK that integrates with Qdrant vector database to answer questions based on the Physical AI & Humanoid Robotics textbook content.

## Prerequisites

- Python 3.11+
- OpenAI API key
- Access to Qdrant vector database (with existing embeddings)
- Required Python packages (openai, qdrant-client, python-dotenv)

## Setup

### 1. Environment Configuration

```bash
# Ensure your .env file contains:
OPENAI_API_KEY="your-openai-api-key"
QDRANT_URL="https://your-qdrant-url"
QDRANT_API_KEY="your-qdrant-api-key"
```

### 2. Install Dependencies

```bash
cd backend/embedding_pipeline
pip install openai qdrant-client python-dotenv
```

## Usage

### Running the Agent

1. **Initialize the Agent**
   ```python
   from agent import OpenAIAgentRAG

   # Initialize with environment variables
   agent = OpenAIAgentRAG()
   ```

2. **Chat with the Agent**
   ```python
   result = agent.chat("What is ROS2?", top_k=5, threshold=0.5)
   print(result['response'])
   ```

3. **Run the Interactive Demo**
   ```bash
   cd backend/embedding_pipeline
   python agent.py
   ```

### API Integration

The agent can be integrated into a FastAPI application:

```python
from fastapi import FastAPI
from agent import OpenAIAgentRAG

app = FastAPI()

@app.post("/api/agent/chat")
async def chat_endpoint(query: str, top_k: int = 5, threshold: float = 0.5):
    agent = OpenAIAgentRAG()
    result = agent.chat(query, top_k=top_k, threshold=threshold)
    return result
```

## Key Components

### QdrantRetrievalTool
- Function tool for retrieving content from Qdrant
- Automatically registered with the OpenAI Assistant
- Handles embedding generation and similarity search

### OpenAIAgentRAG
- Main agent class that manages OpenAI Assistant
- Handles thread creation and destruction
- Orchestrates tool calls and response generation

## Configuration Options

### Agent Parameters
- `top_k`: Number of results to retrieve (default: 5)
- `threshold`: Minimum similarity score (default: 0.5)
- `temperature`: Response creativity (default: 0.7)

### Environment Variables
- `OPENAI_API_KEY`: Required for OpenAI API access
- `QDRANT_URL`: URL for Qdrant database (default: http://localhost:6333)
- `QDRANT_API_KEY`: API key for Qdrant (optional for local instances)

## Expected Behavior

### Successful Queries
- Agent retrieves relevant content from Qdrant
- Response is grounded in retrieved content
- Sources are cited in the response

### Out-of-Context Queries
- Agent politely refuses to answer
- Returns message indicating no relevant content found
- Does not hallucinate information

### Error Handling
- Graceful degradation when APIs are unavailable
- Informative error messages
- Proper cleanup of resources

## Testing

Run the test suite to verify functionality:

```bash
cd backend/embedding_pipeline
python test_agent.py
```

## Integration Points

### With Existing Pipeline
- Uses the same "rag_embedding" collection as the embedding pipeline
- Compatible with existing data structures
- Follows the same retrieval patterns

### Future Frontend Integration
- Stateless per request design
- JSON API responses ready for frontend consumption
- Proper error handling for user feedback