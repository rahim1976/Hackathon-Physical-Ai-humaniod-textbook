"""
Database middleware for the RAG Chatbot API
Handles database connection management for FastAPI requests.
"""

import logging
from typing import Callable, Awaitable

from fastapi import Request, HTTPException
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.requests import Request as StarletteRequest
from starlette.responses import Response

from .database import get_db_manager

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class DatabaseMiddleware(BaseHTTPMiddleware):
    """Middleware to handle database connections for each request."""

    async def dispatch(self, request: Request, call_next: Callable[[Request], Awaitable[Response]]) -> Response:
        """Process the request with database connection management."""
        # Add database manager to request state
        request.state.db = await get_db_manager()

        try:
            response = await call_next(request)
        except Exception as e:
            logger.error(f"Error processing request: {str(e)}")
            raise HTTPException(status_code=500, detail="Internal server error")
        finally:
            # Clean up if needed
            pass

        return response

async def get_db(request: Request):
    """Dependency to get database manager from request state."""
    return request.state.db

# Additional middleware for chat session management
class ChatSessionMiddleware(BaseHTTPMiddleware):
    """Middleware to manage chat sessions automatically."""

    async def dispatch(self, request: StarletteRequest, call_next: Callable[[Request], Awaitable[Response]]) -> Response:
        """Process the request with chat session management."""
        # Extract session ID from headers or cookies
        session_id = request.headers.get('X-Session-ID') or request.cookies.get('session_id')

        if not session_id:
            # Generate a new session ID if none exists
            import uuid
            session_id = str(uuid.uuid4())
            request.state.new_session = True
        else:
            request.state.new_session = False

        request.state.session_id = session_id

        # Add session ID to response headers
        response = await call_next(request)

        if not response.headers.get('X-Session-ID'):
            response.headers['X-Session-ID'] = session_id

        # Set session ID in cookie if not present
        if 'session_id' not in response.headers.get('Set-Cookie', ''):
            response.set_cookie('session_id', session_id, httponly=True, max_age=86400)  # 24 hours

        return response

# Request logging middleware
class RequestLoggingMiddleware(BaseHTTPMiddleware):
    """Middleware to log incoming requests."""

    async def dispatch(self, request: Request, call_next: Callable[[Request], Awaitable[Response]]) -> Response:
        """Log request information."""
        logger.info(f"{request.method} {request.url.path} - {request.client.host}")

        response = await call_next(request)

        logger.info(f"{request.method} {request.url.path} - {response.status_code}")

        return response