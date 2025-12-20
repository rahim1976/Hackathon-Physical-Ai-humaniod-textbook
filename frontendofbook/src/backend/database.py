"""
Database utilities for the RAG Chatbot
Handles database connections and models for Neon Postgres.
"""

import os
import logging
from typing import Optional, List, Dict, Any
from contextlib import asynccontextmanager
from datetime import datetime

import asyncpg
from pydantic import BaseModel

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class ChatSession(BaseModel):
    """Model for chat session data."""
    id: Optional[int] = None
    session_id: str
    user_id: Optional[str] = None
    created_at: datetime
    updated_at: datetime
    metadata: Optional[Dict[str, Any]] = None

class ChatMessage(BaseModel):
    """Model for chat message data."""
    id: Optional[int] = None
    session_id: str
    role: str  # 'user' or 'assistant'
    content: str
    timestamp: datetime
    sources: Optional[List[str]] = None
    metadata: Optional[Dict[str, Any]] = None

class DatabaseManager:
    """Manages database connections and operations for the chatbot."""

    def __init__(self, database_url: str):
        self.database_url = database_url
        self.pool: Optional[asyncpg.Pool] = None

    async def initialize(self):
        """Initialize the database connection pool."""
        try:
            self.pool = await asyncpg.create_pool(
                self.database_url,
                min_size=1,
                max_size=10,
                command_timeout=60
            )
            logger.info("Database connection pool created successfully")

            # Create required tables
            await self._create_tables()
            logger.info("Database tables created/verified successfully")

        except Exception as e:
            logger.error(f"Error initializing database: {str(e)}")
            raise

    async def _create_tables(self):
        """Create required database tables."""
        async with self.pool.acquire() as conn:
            # Create chat_sessions table
            await conn.execute("""
                CREATE TABLE IF NOT EXISTS chat_sessions (
                    id SERIAL PRIMARY KEY,
                    session_id VARCHAR(255) UNIQUE NOT NULL,
                    user_id VARCHAR(255),
                    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    metadata JSONB
                )
            """)

            # Create chat_messages table
            await conn.execute("""
                CREATE TABLE IF NOT EXISTS chat_messages (
                    id SERIAL PRIMARY KEY,
                    session_id VARCHAR(255) NOT NULL,
                    role VARCHAR(50) NOT NULL,
                    content TEXT NOT NULL,
                    timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    sources JSONB,
                    metadata JSONB,
                    FOREIGN KEY (session_id) REFERENCES chat_sessions(session_id) ON DELETE CASCADE
                )
            """)

            # Create indexes for better performance
            await conn.execute("""
                CREATE INDEX IF NOT EXISTS idx_chat_sessions_user_id ON chat_sessions(user_id);
                CREATE INDEX IF NOT EXISTS idx_chat_sessions_created_at ON chat_sessions(created_at);
                CREATE INDEX IF NOT EXISTS idx_chat_messages_session_id ON chat_messages(session_id);
                CREATE INDEX IF NOT EXISTS idx_chat_messages_timestamp ON chat_messages(timestamp);
            """)

            logger.info("Database tables created successfully")

    async def close(self):
        """Close the database connection pool."""
        if self.pool:
            await self.pool.close()
            logger.info("Database connection pool closed")

    async def create_session(self, session_id: str, user_id: Optional[str] = None, metadata: Optional[Dict] = None) -> ChatSession:
        """Create a new chat session."""
        async with self.pool.acquire() as conn:
            try:
                query = """
                    INSERT INTO chat_sessions (session_id, user_id, metadata)
                    VALUES ($1, $2, $3)
                    RETURNING id, session_id, user_id, created_at, updated_at, metadata
                """
                record = await conn.fetchrow(query, session_id, user_id, metadata)

                return ChatSession(
                    id=record['id'],
                    session_id=record['session_id'],
                    user_id=record['user_id'],
                    created_at=record['created_at'],
                    updated_at=record['updated_at'],
                    metadata=record['metadata']
                )
            except asyncpg.UniqueViolationError:
                # Session already exists, fetch it instead
                return await self.get_session(session_id)

    async def get_session(self, session_id: str) -> Optional[ChatSession]:
        """Get a chat session by ID."""
        async with self.pool.acquire() as conn:
            query = """
                SELECT id, session_id, user_id, created_at, updated_at, metadata
                FROM chat_sessions
                WHERE session_id = $1
            """
            record = await conn.fetchrow(query, session_id)

            if record:
                return ChatSession(
                    id=record['id'],
                    session_id=record['session_id'],
                    user_id=record['user_id'],
                    created_at=record['created_at'],
                    updated_at=record['updated_at'],
                    metadata=record['metadata']
                )
            return None

    async def create_message(self, session_id: str, role: str, content: str, sources: Optional[List[str]] = None, metadata: Optional[Dict] = None) -> ChatMessage:
        """Create a new chat message."""
        async with self.pool.acquire() as conn:
            query = """
                INSERT INTO chat_messages (session_id, role, content, sources, metadata)
                VALUES ($1, $2, $3, $4, $5)
                RETURNING id, session_id, role, content, timestamp, sources, metadata
            """
            record = await conn.fetchrow(query, session_id, role, content, sources, metadata)

            return ChatMessage(
                id=record['id'],
                session_id=record['session_id'],
                role=record['role'],
                content=record['content'],
                timestamp=record['timestamp'],
                sources=record['sources'],
                metadata=record['metadata']
            )

    async def get_session_messages(self, session_id: str, limit: int = 50) -> List[ChatMessage]:
        """Get messages for a specific session."""
        async with self.pool.acquire() as conn:
            query = """
                SELECT id, session_id, role, content, timestamp, sources, metadata
                FROM chat_messages
                WHERE session_id = $1
                ORDER BY timestamp ASC
                LIMIT $2
            """
            records = await conn.fetch(query, session_id, limit)

            return [
                ChatMessage(
                    id=record['id'],
                    session_id=record['session_id'],
                    role=record['role'],
                    content=record['content'],
                    timestamp=record['timestamp'],
                    sources=record['sources'],
                    metadata=record['metadata']
                )
                for record in records
            ]

    async def update_session_metadata(self, session_id: str, metadata: Dict[str, Any]) -> bool:
        """Update session metadata."""
        async with self.pool.acquire() as conn:
            query = """
                UPDATE chat_sessions
                SET metadata = $2, updated_at = CURRENT_TIMESTAMP
                WHERE session_id = $1
                RETURNING id
            """
            result = await conn.fetchval(query, session_id, metadata)
            return result is not None

    async def delete_session(self, session_id: str) -> bool:
        """Delete a chat session and all its messages."""
        async with self.pool.acquire() as conn:
            query = "DELETE FROM chat_sessions WHERE session_id = $1"
            result = await conn.execute(query, session_id)
            return result != "DELETE 0"

    async def get_recent_sessions(self, user_id: Optional[str] = None, limit: int = 20) -> List[ChatSession]:
        """Get recent chat sessions, optionally filtered by user."""
        async with self.pool.acquire() as conn:
            if user_id:
                query = """
                    SELECT id, session_id, user_id, created_at, updated_at, metadata
                    FROM chat_sessions
                    WHERE user_id = $1
                    ORDER BY updated_at DESC
                    LIMIT $2
                """
                records = await conn.fetch(query, user_id, limit)
            else:
                query = """
                    SELECT id, session_id, user_id, created_at, updated_at, metadata
                    FROM chat_sessions
                    ORDER BY updated_at DESC
                    LIMIT $2
                """
                records = await conn.fetch(query, limit)

            return [
                ChatSession(
                    id=record['id'],
                    session_id=record['session_id'],
                    user_id=record['user_id'],
                    created_at=record['created_at'],
                    updated_at=record['updated_at'],
                    metadata=record['metadata']
                )
                for record in records
            ]

# Global database manager instance
db_manager: Optional[DatabaseManager] = None

async def get_db_manager() -> DatabaseManager:
    """Get the global database manager instance."""
    global db_manager
    if not db_manager:
        database_url = os.getenv('DATABASE_URL')
        if not database_url:
            raise ValueError("DATABASE_URL environment variable not set")

        db_manager = DatabaseManager(database_url)
        await db_manager.initialize()

    return db_manager