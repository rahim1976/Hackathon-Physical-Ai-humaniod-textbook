"""
FastAPI Backend for OpenRouter Agent Integration

This module implements a FastAPI backend that exposes the OpenRouter agent functionality
through API endpoints, allowing frontend applications to communicate with the retrieval-enabled agent.
"""
import os
import logging
from typing import Dict, Any, Optional
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from dotenv import load_dotenv

from agent import OpenRouterAgentRAG

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Create FastAPI app
app = FastAPI(title="OpenRouter Agent API", description="API for OpenRouter-powered agent with retrieval capabilities")

# Add CORS middleware to allow requests from local frontend
from fastapi.middleware.cors import CORSMiddleware

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, specify exact origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Request model
class ChatRequest(BaseModel):
    message: str
    selected_text: Optional[str] = ""
    context: Optional[str] = ""

# Response model
class ChatResponse(BaseModel):
    response: str
    sources: list[str]
    chunks_used: int
    relevant_chunks: list[dict]
    status: str
    request_id: Optional[str] = None

# Initialize the agent
try:
    agent = OpenRouterAgentRAG()
    logger.info("OpenRouter Agent initialized successfully")
except Exception as e:
    logger.error(f"Failed to initialize OpenRouter Agent: {e}")
    raise

@app.post("/chat", response_model=ChatResponse)
async def chat_endpoint(request: ChatRequest):
    """
    Chat endpoint that processes user queries through the OpenRouter agent.

    Args:
        request: ChatRequest containing the user's message and optional context

    Returns:
        ChatResponse with the agent's response and metadata
    """
    try:
        # Prepare the query - include selected text if provided
        user_query = request.message
        if request.selected_text:
            user_query = f"Context: {request.selected_text}\n\nQuestion: {request.message}"

        # Process the query with the agent
        result = agent.chat(
            user_query=user_query,
            top_k=5,
            threshold=0.4  # Lower threshold to capture more relevant results
        )

        # Format the response
        response = ChatResponse(
            response=result.get("response", ""),
            sources=result.get("sources", []),
            chunks_used=result.get("chunks_used", 0),
            relevant_chunks=result.get("relevant_chunks", []),
            status=result.get("status", "success")
        )

        logger.info("Processed chat request successfully")
        return response

    except Exception as e:
        logger.error(f"Error processing chat request: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error processing request: {str(e)}")

@app.get("/health")
async def health_check():
    """
    Health check endpoint to verify the API is running.
    """
    return {"status": "healthy", "service": "OpenRouter Agent API"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)