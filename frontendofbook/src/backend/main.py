"""
Backend service startup script
This script starts the FastAPI backend service for the RAG chatbot.
"""

import os
import sys
import logging
from typing import Optional

import uvicorn

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)

def start_backend(
    host: str = "0.0.0.0",
    port: int = 8000,
    reload: bool = False,
    workers: int = 1
):
    """
    Start the FastAPI backend service.

    Args:
        host: Host address to bind to
        port: Port to run the service on
        reload: Whether to enable auto-reload for development
        workers: Number of worker processes
    """
    logger.info(f"Starting backend service on {host}:{port}")

    # Set environment variables that the application might need
    os.environ.setdefault('INDEX_DOCUMENTATION', 'false')  # Don't auto-index on startup

    try:
        uvicorn.run(
            "src.api.chatbot_api:app",
            host=host,
            port=port,
            reload=reload,
            workers=workers,
            log_level="info"
        )
    except KeyboardInterrupt:
        logger.info("Backend service stopped by user")
    except Exception as e:
        logger.error(f"Error starting backend service: {str(e)}")
        sys.exit(1)

def main():
    """Main entry point for the backend service."""
    import argparse

    parser = argparse.ArgumentParser(description="AI/Spec-Driven Book RAG Chatbot Backend")
    parser.add_argument(
        "--host",
        type=str,
        default="0.0.0.0",
        help="Host address to bind to (default: 0.0.0.0)"
    )
    parser.add_argument(
        "--port",
        type=int,
        default=8000,
        help="Port to run the service on (default: 8000)"
    )
    parser.add_argument(
        "--reload",
        action="store_true",
        help="Enable auto-reload for development (default: False)"
    )
    parser.add_argument(
        "--workers",
        type=int,
        default=1,
        help="Number of worker processes (default: 1)"
    )

    args = parser.parse_args()

    logger.info("Starting AI/Spec-Driven Book RAG Chatbot Backend")
    logger.info(f"Configuration: host={args.host}, port={args.port}, reload={args.reload}, workers={args.workers}")

    start_backend(
        host=args.host,
        port=args.port,
        reload=args.reload,
        workers=args.workers
    )

if __name__ == "__main__":
    main()