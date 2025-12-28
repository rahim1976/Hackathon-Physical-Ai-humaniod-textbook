"""
Debug version of the OpenRouter Agent to help identify API compatibility issues.
"""
import os
import logging
from typing import Dict, List, Any, Optional
from pydantic import BaseModel, Field
import json

import openai
from openai import OpenAI
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Import the retrieval functionality from the existing file
from retrieving import RAGRetriever

class QdrantRetrievalTool(BaseModel):
    """
    Tool for retrieving relevant chunks from Qdrant vector database.
    This tool can be used by the OpenRouter Agent to fetch relevant content.
    """
    name: str = Field(default="retrieve_content", description="Name of the tool")
    description: str = Field(
        default="Retrieve relevant content from the Physical AI & Humanoid Robotics textbook based on user query",
        description="Description of the tool"
    )
    query: str = Field(..., description="The user's query to search for relevant content")
    top_k: int = Field(default=5, description="Number of top results to return")
    threshold: float = Field(default=0.4, description="Minimum similarity score threshold")

    def retrieve_relevant_content(self) -> List[Dict[str, Any]]:
        """
        Retrieve relevant content from Qdrant based on the query.
        """
        try:
            # Initialize the RAGRetriever from the existing module
            retriever = RAGRetriever()

            # Use the existing retrieve_context function
            results = retriever.retrieve_context(
                query=self.query,
                k=self.top_k,
                threshold=self.threshold
            )

            logger.info(f"Retrieved {len(results)} relevant chunks for query: {self.query}")
            return results

        except Exception as e:
            logger.error(f"Error retrieving content from Qdrant: {str(e)}")
            return []


class OpenRouterAgentRAG:
    """
    A retrieval-enabled agent using OpenAI Agents SDK that integrates with OpenRouter.
    """

    def __init__(self,
                 openrouter_api_key: Optional[str] = None,
                 qdrant_url: Optional[str] = None,
                 qdrant_api_key: Optional[str] = None,
                 agent_instructions: Optional[str] = None):

        # Get API keys from parameters or environment
        self.openrouter_api_key = openrouter_api_key or os.getenv("OPENROUTER_API_KEY")
        self.qdrant_url = qdrant_url or os.getenv("QDRANT_URL", "http://localhost:6333")
        self.qdrant_api_key = qdrant_api_key or os.getenv("QDRANT_API_KEY")

        if not self.openrouter_api_key:
            raise ValueError("OPENROUTER_API_KEY is required")

        # Initialize OpenAI client with OpenRouter base URL
        self.openrouter_client = OpenAI(
            api_key=self.openrouter_api_key,
            base_url="https://openrouter.ai/api/v1"
        )

        # Set up default agent instructions if not provided
        if agent_instructions is None:
            agent_instructions = """
            You are a professional technical assistant. Use the retrieval tool to answer questions based on the book.
            If no relevant info is found, state that the information is missing from the documentation.
            """

        self.agent_instructions = agent_instructions

        # Create the agent
        self.agent = self._create_agent()

        logger.info("OpenRouter Agent with retrieval integration initialized successfully")

    def _create_agent(self):
        """
        Create an OpenRouter assistant with the retrieval tool.
        """
        # Define the tool that the agent can use
        tools = [{
            "type": "function",
            "function": {
                "name": "retrieve_content",
                "description": "Retrieve relevant content from the Physical AI & Humanoid Robotics textbook based on user query",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "query": {
                            "type": "string",
                            "description": "The user's query to search for relevant content"
                        },
                        "top_k": {
                            "type": "integer",
                            "description": "Number of top results to return (default: 5)",
                            "default": 5
                        },
                        "threshold": {
                            "type": "number",
                            "description": "Minimum similarity score threshold (default: 0.4)",
                            "default": 0.4
                        }
                    },
                    "required": ["query"]
                }
            }
        }]

        # Create the assistant using OpenRouter model
        assistant = self.openrouter_client.beta.assistants.create(
            name="Physical AI & Humanoid Robotics RAG Agent (OpenRouter)",
            description="An AI assistant that answers questions about robotics, ROS2, humanoid design, and related topics using retrieved content from the textbook",
            model="mistralai/devstral-2512:free",  # Using the specified free model
            instructions=self.agent_instructions,
            tools=tools
        )

        return assistant

    def chat(self, user_query: str, top_k: int = 5, threshold: float = 0.4) -> Dict[str, Any]:
        """
        Main chat method that uses the OpenRouter Agent to answer user queries.

        Args:
            user_query: The user's question
            top_k: Number of relevant chunks to retrieve
            threshold: Minimum similarity threshold

        Returns:
            Dictionary containing the agent's response and metadata
        """
        try:
            # Create a thread for the conversation
            thread_response = self.openrouter_client.beta.threads.create(
                messages=[
                    {
                        "role": "user",
                        "content": user_query
                    }
                ]
            )

            # Get the thread ID - handle both object and string responses
            if hasattr(thread_response, 'id'):
                thread_id = thread_response.id
                logger.info(f"Thread created with ID: {thread_id}")
            else:
                thread_id = str(thread_response)
                logger.warning(f"Thread response was not an object, converted to string: {thread_id}")

            # Run the agent
            run_response = self.openrouter_client.beta.threads.runs.create(
                thread_id=thread_id,
                assistant_id=self.agent.id
            )

            # Get the run ID - handle both object and string responses
            if hasattr(run_response, 'id'):
                run_id = run_response.id
                logger.info(f"Run created with ID: {run_id}")
            else:
                run_id = str(run_response)
                logger.warning(f"Run response was not an object, converted to string: {run_id}")

            # Wait for the run to complete
            import time
            run = run_response  # Initialize run object
            loop_count = 0
            max_loops = 20  # Prevent infinite loops
            while run.status in ["queued", "in_progress"] and loop_count < max_loops:
                loop_count += 1
                time.sleep(2)  # Increased delay to allow for processing
                run = self.openrouter_client.beta.threads.runs.retrieve(
                    thread_id=thread_id,
                    run_id=run_id
                )

                logger.info(f"Run status: {run.status}")

                # If the run requires action (tool call), handle it
                if hasattr(run, 'status') and run.status == "requires_action":
                    logger.info("Run requires action - processing tool calls")
                    # Process tool calls
                    tool_outputs = []
                    # Handle different possible formats for tool calls
                    required_action = getattr(run, 'required_action', None)
                    if required_action:
                        submit_tool_outputs = getattr(required_action, 'submit_tool_outputs', None)
                        if submit_tool_outputs:
                            tool_calls = getattr(submit_tool_outputs, 'tool_calls', [])
                            logger.info(f"Found {len(tool_calls)} tool calls to process")
                            for tool_call in tool_calls:
                                # Check if tool_call has the expected attributes
                                if hasattr(tool_call, 'function'):
                                    function = getattr(tool_call, 'function', None)
                                    if function and hasattr(function, 'name') and getattr(function, 'name', '') == "retrieve_content":
                                        logger.info(f"Processing tool call: {getattr(tool_call, 'id', 'unknown')}")
                                        # Parse the function arguments
                                        import json
                                        args_str = getattr(function, 'arguments', '{}')
                                        try:
                                            args = json.loads(args_str)
                                        except json.JSONDecodeError:
                                            args = {}
                                            logger.warning(f"Failed to parse arguments: {args_str}")

                                        # Create the retrieval tool and call it
                                        retrieval_tool = QdrantRetrievalTool(
                                            query=args.get("query", user_query),
                                            top_k=args.get("top_k", top_k),
                                            threshold=args.get("threshold", threshold)
                                        )

                                        results = retrieval_tool.retrieve_relevant_content()

                                        # Format the results for the agent
                                        tool_output = {
                                            "tool_call_id": getattr(tool_call, 'id', ''),
                                            "output": json.dumps(results)
                                        }
                                        tool_outputs.append(tool_output)
                                        logger.info(f"Added tool output for call ID: {tool_output['tool_call_id']}")

                    if tool_outputs:
                        logger.info(f"Submitting {len(tool_outputs)} tool outputs")
                        # Submit the tool outputs back to the agent
                        submit_response = self.openrouter_client.beta.threads.runs.submit_tool_outputs(
                            thread_id=thread_id,
                            run_id=run_id,
                            tool_outputs=tool_outputs
                        )
                        # Update run object with the response from tool submission
                        run = submit_response
                        logger.info(f"Tool outputs submitted, new run status: {run.status}")
                    else:
                        logger.warning("No tool outputs to submit")
                        break

            # After the loop, check the final run status
            logger.info(f"Final run status: {run.status}")

            # Get the messages from the thread
            messages_response = self.openrouter_client.beta.threads.messages.list(
                thread_id=thread_id,
                order="desc"
            )

            # Extract the agent's response
            response_text = ""
            # Handle both object and data array responses
            messages_data = getattr(messages_response, 'data', messages_response)
            if isinstance(messages_data, list):
                for message in messages_data:
                    if hasattr(message, 'role') and getattr(message, 'role', '') == "assistant":
                        content_items = getattr(message, 'content', [])
                        if content_items and len(content_items) > 0:
                            text_item = content_items[0]
                            if hasattr(text_item, 'text'):
                                response_text = getattr(text_item.text, 'value', str(text_item))
                            else:
                                response_text = str(text_item)
                            break
                    elif isinstance(message, dict) and message.get('role') == 'assistant':
                        content_items = message.get('content', [])
                        if content_items and len(content_items) > 0:
                            text_item = content_items[0]
                            response_text = text_item.get('text', {}).get('value', str(text_item))
                            break
            else:
                logger.warning(f"messages_data is not a list, type: {type(messages_data)}")

            # Also retrieve the tool call results if needed for metadata
            relevant_chunks = []
            # Note: In a real implementation, we'd want to capture the actual retrieved chunks
            # For now, we'll do a separate retrieval to get the chunks for metadata

            # Create retrieval tool to get the actual chunks for metadata
            retrieval_tool = QdrantRetrievalTool(
                query=user_query,
                top_k=top_k,
                threshold=threshold
            )
            relevant_chunks = retrieval_tool.retrieve_relevant_content()

            # Extract source information
            sources = list(set([chunk["url"] for chunk in relevant_chunks if chunk["url"]]))

            # Clean up the thread
            try:
                self.openrouter_client.beta.threads.delete(thread_id)
                logger.info(f"Thread {thread_id} deleted successfully")
            except Exception as e:
                logger.warning(f"Failed to delete thread {thread_id}: {str(e)}")

            result = {
                "response": response_text,
                "sources": sources,
                "chunks_used": len(relevant_chunks),
                "relevant_chunks": relevant_chunks,
                "status": "success"
            }

            logger.info(f"Agent successfully processed query: {user_query[:50]}...")
            return result

        except Exception as e:
            logger.error(f"Error in agent chat: {str(e)}")
            import traceback
            logger.error(f"Full traceback: {traceback.format_exc()}")
            return {
                "response": "I encountered an error processing your query. Please try again.",
                "sources": [],
                "chunks_used": 0,
                "status": "error",
                "error": str(e)
            }

    def __del__(self):
        """
        Cleanup method to delete the assistant when the object is destroyed.
        """
        try:
            if hasattr(self, 'agent'):
                self.openrouter_client.beta.assistants.delete(self.agent.id)
        except:
            pass  # Ignore errors during cleanup


def main():
    """
    Main function to demonstrate the OpenRouter Agent with retrieval integration.
    """
    logger.info("Initializing OpenRouter Agent with retrieval integration...")

    # Initialize the agent
    agent = OpenRouterAgentRAG()

    print("OpenRouter Agent with Retrieval Integration - Debug Version")
    print("=" * 60)
    print("Ask questions about robotics, ROS2, humanoid design, etc.")
    print("Type 'quit' to exit")
    print()

    while True:
        user_input = input("Your question: ").strip()

        if user_input.lower() in ['quit', 'exit', 'q']:
            print("Goodbye!")
            break

        if not user_input:
            continue

        print("\nProcessing your query...")
        result = agent.chat(user_input)

        print(f"\nResponse: {result['response']}")

        if result['sources']:
            print(f"\nSources: {len(result['sources'])} documents referenced")
            for i, source in enumerate(result['sources'][:3], 1):  # Show first 3 sources
                print(f"  {i}. {source}")

        print("-" * 60)


if __name__ == "__main__":
    main()