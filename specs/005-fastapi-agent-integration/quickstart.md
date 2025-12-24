# Quickstart Guide: OpenAI Agent with OpenRouter Integration

**Feature**: OpenAI Agent with OpenRouter Integration
**Created**: 2025-12-21

## Overview

This guide helps you set up and use the retrieval-enabled agent using OpenAI Agents SDK that integrates with OpenRouter API and the existing Qdrant retrieval system. The agent uses the free mistralai/devstral-2512:free model through OpenRouter.

## Prerequisites

- Python 3.11+
- OpenRouter API key for the free tier
- Access to Qdrant vector database (with existing embeddings)
- Required Python packages (openai, qdrant-client, python-dotenv)

## Setup

### 1. Environment Configuration

```bash
# Ensure your .env file contains:
OPENROUTER_API_KEY="your-openrouter-api-key"
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
   from agent import OpenRouterAgentRAG

   # Initialize with environment variables
   agent = OpenRouterAgentRAG()
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

### API Integration (Future)

The agent can be integrated into a FastAPI application:

```python
from fastapi import FastAPI
from agent import OpenRouterAgentRAG

app = FastAPI()

@app.post("/api/agent/chat")
async def chat_endpoint(query: str, top_k: int = 5, threshold: float = 0.5):
    agent = OpenRouterAgentRAG()
    result = agent.chat(query, top_k=top_k, threshold=threshold)
    return result
```

## Key Components

### QdrantRetrievalTool
- Function tool for retrieving content from Qdrant
- Automatically registered with the OpenRouter Assistant
- Handles embedding generation and similarity search

### OpenRouterAgentRAG
- Main agent class that manages OpenRouter Assistant
- Handles thread creation and destruction
- Orchestrates tool calls and response generation

## Configuration Options

### Agent Parameters
- `top_k`: Number of results to retrieve (default: 5)
- `threshold`: Minimum similarity score (default: 0.5)
- `temperature`: Response creativity (default: 0.7)

### Environment Variables
- `OPENROUTER_API_KEY`: Required for OpenRouter API access
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

### Future FastAPI Integration
- Stateless per request design
- JSON API responses ready for frontend consumption
- Proper error handling for user feedback