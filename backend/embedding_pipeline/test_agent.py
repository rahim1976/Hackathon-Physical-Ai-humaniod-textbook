"""
Test script for the OpenRouter Agent with retrieval integration.
"""
import os
import sys
import logging

# Add the current directory to the path so we can import the agent
sys.path.insert(0, os.path.dirname(__file__))

from agent import OpenRouterAgentRAG

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def test_agent_initialization():
    """Test that the agent can be initialized successfully."""
    print("Testing agent initialization...")

    try:
        # Initialize the agent
        agent = OpenRouterAgentRAG()
        print("[PASS] Agent initialized successfully")
        return True
    except Exception as e:
        print(f"[FAIL] Agent initialization failed: {e}")
        return False

def test_retrieval_functionality():
    """Test that the retrieval functionality works."""
    print("\nTesting retrieval functionality...")

    try:
        # Initialize the agent
        agent = OpenRouterAgentRAG()

        # Test the retrieval function directly
        results = agent.retrieve_content("What is ROS2?", top_k=3, threshold=0.5)
        print(f"[PASS] Retrieval functionality executed successfully, found {len(results)} results")

        if results:
            print(f"First result similarity score: {results[0]['similarity_score']:.3f}")
            print(f"First result content preview: {results[0]['content'][:100]}...")

        return True
    except Exception as e:
        print(f"[FAIL] Retrieval functionality failed: {e}")
        return False

def test_agent_chat():
    """Test the agent's chat functionality."""
    print("\nTesting agent chat functionality...")

    try:
        agent = OpenRouterAgentRAG()

        # Test with a simple query
        result = agent.chat("What is this textbook about?", top_k=3, threshold=0.5)

        print(f"[PASS] Agent chat executed successfully")
        print(f"Response preview: {result['response'][:100]}...")
        print(f"Sources found: {len(result['sources'])}")
        print(f"Chunks used: {result['chunks_used']}")

        return True
    except Exception as e:
        print(f"[FAIL] Agent chat failed: {e}")
        return False

def main():
    """Run all tests."""
    print("Running tests for OpenRouter Agent with retrieval integration")
    print("=" * 60)

    tests = [
        test_agent_initialization,
        test_retrieval_functionality,
        test_agent_chat
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
    else:
        print("Some tests failed. Check the output above for details.")

if __name__ == "__main__":
    main()