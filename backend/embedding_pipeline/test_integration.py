"""
Test script for the OpenRouter Agent FastAPI integration.
"""
import os
import sys
import logging

# Add the current directory to the path so we can import the modules
sys.path.insert(0, os.path.dirname(__file__))

from main import app, agent
from agent import OpenRouterAgentRAG

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def test_agent_initialization():
    """Test that the agent can be initialized successfully."""
    print("Testing agent initialization...")

    try:
        # Check if agent is already initialized in main.py
        assert agent is not None, "Agent should be initialized"
        print("[PASS] Agent initialized successfully")
        return True
    except Exception as e:
        print(f"[FAIL] Agent initialization failed: {e}")
        return False

def test_agent_functionality():
    """Test that the agent can process queries successfully."""
    print("\nTesting agent functionality...")

    try:
        # Test the agent directly
        result = agent.chat("What is this textbook about?", top_k=3, threshold=0.5)

        print("[PASS] Agent chat executed successfully")
        print(f"Response preview: {result['response'][:100]}...")
        print(f"Sources found: {len(result['sources'])}")
        print(f"Chunks used: {result['chunks_used']}")

        return True
    except Exception as e:
        print(f"[FAIL] Agent functionality failed: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_retrieval_integration():
    """Test that the retrieval system is properly integrated."""
    print("\nTesting retrieval integration...")

    try:
        # Test that the agent can retrieve content
        results = agent.retrieve_content("What is ROS2?", top_k=3, threshold=0.5)

        print(f"[PASS] Retrieval system executed successfully, found {len(results)} results")
        if results:
            print(f"First result similarity score: {results[0]['similarity_score']:.3f}")
            print(f"First result content preview: {results[0]['content'][:100]}...")

        return True
    except Exception as e:
        print(f"[FAIL] Retrieval integration failed: {e}")
        import traceback
        traceback.print_exc()
        return False

def main():
    """Run all tests."""
    print("Running tests for OpenRouter Agent FastAPI integration")
    print("=" * 60)

    tests = [
        test_agent_initialization,
        test_agent_functionality,
        test_retrieval_integration
    ]

    passed = 0
    total = len(tests)

    for test in tests:
        if test():
            passed += 1

    print("\n" + "=" * 60)
    print(f"Test Results: {passed}/{total} tests passed")

    if passed == total:
        print("All tests passed!")
        print("\nYou can now run the FastAPI server with:")
        print("  cd backend/embedding_pipeline")
        print("  uvicorn main:app --reload --port 8000")
        print("\nAnd access the API at http://localhost:8000/docs")
    else:
        print("⚠️  Some tests failed. Check the output above for details.")

    return passed == total

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)