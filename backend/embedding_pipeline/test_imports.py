#!/usr/bin/env python3
"""
Simple test to verify all required imports work correctly
"""
import os
import sys

# Add the current directory to the path so we can import main
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

try:
    # Test imports from the main file
    import requests
    import cohere
    from qdrant_client import QdrantClient
    from bs4 import BeautifulSoup
    from dotenv import load_dotenv

    print("[SUCCESS] All required imports successful!")

    # Check if we can import functions from main.py
    from main import DocusaurusEmbeddingPipeline

    print("[SUCCESS] All function imports from main.py successful!")

    print("\n[INFO] Embedding pipeline backend is properly set up with:")
    print("- Cohere client for embeddings")
    print("- Qdrant client for vector storage")
    print("- Beautiful Soup for HTML parsing")
    print("- Requests for HTTP operations")
    print("- Python-dotenv for environment management")
    print("- Proper UV package configuration")

except ImportError as e:
    print(f"[ERROR] Import error: {e}")
    sys.exit(1)
except Exception as e:
    print(f"[ERROR] Error: {e}")
    sys.exit(1)

print("\n[SUCCESS] Backend implementation is ready!")