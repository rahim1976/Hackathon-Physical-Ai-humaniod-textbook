# Quickstart Guide: FastAPI & Floating Chat UI Integration

**Feature**: FastAPI & Floating Chat UI Integration
**Created**: 2025-12-21

## Overview

This guide helps you set up and use the FastAPI backend with floating chat widget that integrates with the OpenRouter Agent for retrieval-augmented responses based on book content.

## Prerequisites

- Python 3.11+
- OpenRouter API key (stored in environment variables)
- Qdrant vector database with existing embeddings
- Required Python packages (fastapi, uvicorn, openai, qdrant-client)
- Node.js (for frontend development, if needed)

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
pip install fastapi uvicorn python-dotenv openai qdrant-client
```

## Usage

### Running the FastAPI Server

1. **Start the Backend Server**
   ```bash
   cd backend/embedding_pipeline
   uvicorn main:app --host 0.0.0.0 --port 8000 --reload
   ```

2. **Verify API Endpoint**
   - Navigate to `http://localhost:8000/docs` to view the API documentation
   - Test the `/chat` endpoint with sample requests

### Using the Floating Chat Widget

1. **Integrate the Widget**:
   - Include the chat widget HTML/CSS/JS in your frontend
   - The widget will appear as a floating button in the bottom-right corner

2. **Widget Features**:
   - Click the floating icon to open/close the chat window
   - Type messages in the input bar and press Enter or click Send
   - Selected text on the page will be automatically included in queries
   - View agent responses with source citations in the chat bubbles

### API Integration

The chat API endpoint accepts requests in the following format:

```json
{
  "message": "user query text",
  "selected_text": "text selected by user (optional)",
  "context": "additional context (optional)"
}
```

And returns responses in this format:

```json
{
  "response": "agent response text",
  "sources": ["url1", "url2"],
  "chunks_used": 3,
  "relevant_chunks": [...],
  "status": "success",
  "request_id": "unique_request_id"
}
```

## Key Components

### FastAPI Server (main.py)
- `/chat` POST endpoint for processing user queries
- CORS middleware configured for local development
- Integration with OpenRouter Agent for responses
- Proper error handling and logging

### Floating Chat Widget
- Fixed-position floating button (bottom-right)
- Toggleable chat window with message history
- Input field for sending queries
- Selected text capture functionality
- Loading indicators during agent processing

### Agent Integration
- Uses existing agent.py with OpenRouter API
- Processes queries with retrieval capabilities
- Returns grounded responses based on book content
- Includes source citations in responses

## Configuration Options

### Server Configuration
- `PORT`: Port number for FastAPI server (default: 8000)
- `HOST`: Host address for FastAPI server (default: 0.0.0.0)
- `RELOAD`: Auto-reload on code changes (default: true for development)

### Widget Configuration
- `POSITION`: Floating position (default: bottom-right)
- `SIZE`: Widget dimensions (default: responsive)
- `THEME`: Color theme options (default: light/dark auto)

## Expected Behavior

### Successful Queries
- Agent retrieves relevant content from Qdrant
- Response is grounded in retrieved content
- Sources are cited in the response
- Widget shows smooth loading states

### Selected Text Integration
- Text selection on page is captured automatically
- Selected text is included in the prompt context
- User can see when text has been captured
- Agent incorporates selected context into response

### Error Handling
- Graceful degradation when APIs are unavailable
- Informative error messages in the chat
- Proper cleanup of resources
- Maintained user session state

## Testing

### API Testing
```bash
# Test the chat endpoint directly
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{"message": "What is ROS2?", "selected_text": ""}'
```

### Widget Testing
- Click the floating icon to open the chat window
- Send a message and verify the response appears
- Select text on the page and send a query to test context injection
- Verify source citations appear in the response

## Integration Points

### With Existing Pipeline
- Uses the same Qdrant collection as the embedding pipeline
- Compatible with existing data structures
- Follows the same retrieval patterns

### Frontend Integration
- Simple HTML/CSS/JS inclusion
- No framework dependencies required
- Configurable positioning and styling
- Event-based communication with backend