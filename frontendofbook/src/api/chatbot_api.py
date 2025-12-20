"""
FastAPI backend for the RAG Chatbot
Provides REST API endpoints for chatbot functionality.
"""

from fastapi import FastAPI, HTTPException, Depends, BackgroundTasks
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field
from typing import List, Dict, Any, Optional
import asyncio
import logging
import uuid

from src.chatbot.main import ChatbotApplication
from src.chatbot.config import load_config_from_env
from src.backend.database import get_db, ChatMessage
from src.backend.middleware import DatabaseMiddleware, ChatSessionMiddleware

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Initialize FastAPI app
app = FastAPI(
    title="AI/Spec-Driven Book RAG Chatbot API",
    description="API for the RAG chatbot system that provides contextual answers based on documentation",
    version="1.0.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, specify exact origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Pydantic models for request/response
class ChatRequest(BaseModel):
    """Request model for chat endpoint."""
    query: str = Field(..., min_length=1, max_length=2000, description="User query")
    top_k: int = Field(default=5, ge=1, le=20, description="Number of relevant chunks to retrieve")
    threshold: float = Field(default=0.5, ge=0.0, le=1.0, description="Similarity threshold")
    max_tokens: int = Field(default=1000, ge=100, le=4000, description="Max tokens in response")
    temperature: float = Field(default=0.7, ge=0.0, le=2.0, description="Response temperature")

class ChatResponse(BaseModel):
    """Response model for chat endpoint."""
    response: str
    sources: List[str]
    chunks_used: int
    relevant_chunks: Optional[List[Dict[str, Any]]] = None
    query: str

class IndexRequest(BaseModel):
    """Request model for indexing endpoint."""
    content_path: Optional[str] = Field(default=None, description="Path to documentation to index")
    force_reindex: bool = Field(default=False, description="Whether to force reindexing")

class IndexResponse(BaseModel):
    """Response model for indexing endpoint."""
    success: bool
    indexed_count: int
    total_documents: int
    message: str

# Global application instance
chatbot_app: Optional[ChatbotApplication] = None

@app.on_event("startup")
async def startup_event():
    """Initialize the chatbot application on startup."""
    global chatbot_app
    try:
        config = load_config_from_env()
        chatbot_app = ChatbotApplication(config)
        logger.info("Chatbot application initialized successfully")

        # Initialize database
        db = await get_db()
        logger.info("Database initialized successfully")
    except Exception as e:
        logger.error(f"Failed to initialize chatbot application: {str(e)}")
        raise

# Add middleware for database and session management
app.add_middleware(DatabaseMiddleware)
app.add_middleware(ChatSessionMiddleware)

@app.get("/")
async def root():
    """Root endpoint for health check."""
    return {"message": "AI/Spec-Driven Book RAG Chatbot API", "status": "healthy"}

@app.post("/chat", response_model=ChatResponse)
async def chat_endpoint(request: ChatRequest, db=Depends(get_db)):
    """
    Chat endpoint that processes user queries through the RAG system.

    Args:
        request: ChatRequest containing the query and parameters
        db: Database manager dependency

    Returns:
        ChatResponse with the answer and metadata
    """
    if not chatbot_app:
        raise HTTPException(status_code=500, detail="Chatbot application not initialized")

    try:
        # Get session ID from request state (set by middleware)
        session_id = request.state.session_id

        # Store user message in database
        await db.create_message(
            session_id=session_id,
            role="user",
            content=request.query
        )

        # Process the chat request
        result = chatbot_app.chat(query=request.query)

        # Store assistant response in database
        await db.create_message(
            session_id=session_id,
            role="assistant",
            content=result["response"],
            sources=result["sources"]
        )

        # Return the response
        return ChatResponse(
            response=result["response"],
            sources=result["sources"],
            chunks_used=result["chunks_used"],
            relevant_chunks=result.get("relevant_chunks", []),
            query=request.query
        )
    except Exception as e:
        logger.error(f"Error processing chat request: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error processing chat request: {str(e)}")

@app.post("/index", response_model=IndexResponse)
async def index_endpoint(request: IndexRequest):
    """
    Index endpoint that indexes documentation content.

    Args:
        request: IndexRequest containing indexing parameters

    Returns:
        IndexResponse with indexing results
    """
    if not chatbot_app:
        raise HTTPException(status_code=500, detail="Chatbot application not initialized")

    try:
        # Perform indexing
        success = chatbot_app.index_documentation(content_path=request.content_path)

        # For now, we'll return a simple success message
        # In a real implementation, you'd want to track actual counts
        return IndexResponse(
            success=success,
            indexed_count=0,  # Would need to track this in the actual implementation
            total_documents=0,  # Would need to track this in the actual implementation
            message="Indexing completed" if success else "Indexing failed"
        )
    except Exception as e:
        logger.error(f"Error during indexing: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error during indexing: {str(e)}")

@app.get("/health")
async def health_check():
    """Health check endpoint."""
    return {
        "status": "healthy",
        "chatbot_initialized": chatbot_app is not None
    }

@app.get("/config")
async def get_config():
    """Return current configuration (excluding sensitive data)."""
    if not chatbot_app:
        raise HTTPException(status_code=500, detail="Chatbot application not initialized")

    # Return non-sensitive configuration information
    return {
        "chatbot_model": chatbot_app.config.chatbot_model,
        "embedding_model": chatbot_app.config.embedding_model,
        "collection_name": chatbot_app.config.collection_name,
        "max_chunk_size": chatbot_app.config.max_chunk_size,
        "chunk_overlap": chatbot_app.config.chunk_overlap,
        "retrieval_top_k": chatbot_app.config.retrieval_top_k,
        "retrieval_threshold": chatbot_app.config.retrieval_threshold
    }

# Additional endpoints for session management
@app.get("/sessions")
async def get_sessions(user_id: Optional[str] = None, limit: int = 20, db=Depends(get_db)):
    """Get chat sessions, optionally filtered by user."""
    try:
        sessions = await db.get_recent_sessions(user_id=user_id, limit=limit)
        return {"sessions": [session.dict() for session in sessions]}
    except Exception as e:
        logger.error(f"Error getting sessions: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error getting sessions: {str(e)}")

@app.get("/sessions/{session_id}/messages")
async def get_session_messages(session_id: str, limit: int = 50, db=Depends(get_db)):
    """Get messages for a specific session."""
    try:
        messages = await db.get_session_messages(session_id=session_id, limit=limit)
        return {"messages": [message.dict() for message in messages]}
    except Exception as e:
        logger.error(f"Error getting session messages: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error getting session messages: {str(e)}")

@app.delete("/sessions/{session_id}")
async def delete_session(session_id: str, db=Depends(get_db)):
    """Delete a chat session."""
    try:
        success = await db.delete_session(session_id=session_id)
        if success:
            return {"message": "Session deleted successfully"}
        else:
            raise HTTPException(status_code=404, detail="Session not found")
    except Exception as e:
        logger.error(f"Error deleting session: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error deleting session: {str(e)}")

# Additional endpoints for specific functionality
@app.post("/validate-response")
async def validate_response_endpoint(request: ChatResponse):
    """
    Validate that a response is properly grounded in documentation.

    Args:
        request: ChatResponse to validate

    Returns:
        Validation result
    """
    if not chatbot_app:
        raise HTTPException(status_code=500, detail="Chatbot application not initialized")

    try:
        is_valid = chatbot_app.validate_response_grounding(
            query=request.query,
            response=request.response
        )

        return {
            "valid": is_valid,
            "message": "Response is properly grounded" if is_valid else "Response may contain hallucinations"
        }
    except Exception as e:
        logger.error(f"Error validating response: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error validating response: {str(e)}")

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)