# Implementation Tasks: OpenRouter Agent with Retrieval Integration

**Feature**: OpenRouter Agent with Retrieval Integration
**Branch**: 005-fastapi-agent-integration
**Generated**: 2025-12-21

## Implementation Strategy

This feature creates a standalone agent.py file that uses the OpenAI Agents SDK to build a retrieval-enabled agent using OpenRouter API with the mistralai/devstral-2512:free model. The agent integrates with the existing retrieval function as a tool.

**MVP Scope**: Focus on Core Agent Functionality (T001-T010)

## Phase 1: Setup

**Goal**: Initialize agent construction tasks for OpenRouter Agent with retrieval integration

- [x] T001 Verify existing retrieving.py file and retrieve_context function
- [x] T002 Confirm OPENROUTER_API_KEY environment variable availability
- [x] T003 Install required dependencies (openai, python-dotenv)
- [x] T004 Set up development environment for OpenRouter integration

## Phase 2: Foundational

**Goal**: Create foundational components for agent integration

- [x] T005 Create agent.py file with proper imports and setup
- [x] T006 Configure OpenAI client with OpenRouter base URL
- [x] T007 Implement environment variable validation for API keys
- [x] T008 Add comprehensive logging for agent operations

## Phase 3: [US1] Environment Setup and Client Configuration

**Goal**: Ensure proper environment setup and OpenRouter client initialization

**Independent Test**: Can initialize OpenAI client with OpenRouter base URL and API key

- [x] T009 [US1] Install openai and python-dotenv dependencies
- [x] T010 [US1] Load OPENROUTER_API_KEY from environment variables
- [x] T011 [US1] Configure OpenAI client with OpenRouter base URL
- [x] T012 [US1] Validate OpenRouter API connectivity

## Phase 4: [US2] Retrieval Logic Integration

**Goal**: Import and integrate the existing retrieval logic with the agent

**Independent Test**: Can import retrieve_context function and call it successfully

- [x] T013 [US2] Import retrieve_context function from retrieving.py
- [x] T014 [US2] Test retrieval function independently
- [x] T015 [US2] Validate retrieval function parameters and return format
- [x] T016 [US2] Ensure retrieval function works with environment configuration

## Phase 5: [US3] Tool Decoration and Registration

**Goal**: Wrap the retrieve_context function with @function_tool decorator and register with the agent

**Independent Test**: Can initialize the agent with the retrieval tool and the tool is properly recognized

- [x] T017 [US3] Create QdrantRetrievalTool class that wraps retrieve_context functionality
- [x] T018 [US3] Implement proper JSON schema generation for the retrieval tool
- [x] T019 [US3] Register the retrieval tool with the OpenRouter Assistant
- [x] T020 [US3] Validate that the tool can be called by the agent

## Phase 6: [US4] Agent Definition and Instructions

**Goal**: Initialize Agent with the retrieval tool list and system instructions

**Independent Test**: Can create an agent that follows the rules "Use the retrieval tool for every query. If no relevant info found, state that information is missing."

- [x] T021 [US4] Define OpenRouter Assistant with retrieval tool list
- [x] T022 [US4] Add system instructions requiring retrieval tool usage for every query
- [x] T023 [US4] Implement instructions to state when info is missing from documentation
- [x] T024 [US4] Configure agent to use mistralai/devstral-2512:free model

## Phase 7: [US5] Runner Implementation and Testing

**Goal**: Implement runner pattern and test agent functionality

**Independent Test**: Can execute agent runs that properly trigger tools and return grounded responses

- [x] T025 [US5] Implement main execution block for agent testing
- [x] T026 [US5] Create test function to verify tool triggering
- [x] T027 [US5] Validate that responses include book metadata (URLs/headings)
- [x] T028 [US5] Test stateless architecture (no local session files)

## Phase 8: Testing and Validation

**Goal**: Implement comprehensive testing to validate all acceptance criteria

**Independent Test**: Agent imports retrieval function, routes through OpenRouter, includes metadata, and maintains stateless architecture

- [x] T029 [US1] Test that agent.py successfully imports and calls retrieval function
- [x] T030 [US1] Test that agent correctly routes through OpenRouter's API
- [x] T031 [US2] Test that final output includes book metadata (URLs/headings)
- [x] T032 [US3] Test that agent maintains stateless architecture
- [x] T033 Create comprehensive test suite for agent functionality

## Phase 9: CLI Interface & Documentation

**Goal**: Update command-line interface and documentation for agent functionality

- [x] T034 Update help text and usage examples for agent functionality
- [x] T035 Document OpenRouter integration and configuration
- [x] T036 Update quickstart guide with OpenRouter usage examples
- [x] T037 Create API documentation based on implementation

## Phase 10: Polish & Cross-Cutting Concerns

**Goal**: Complete implementation with quality improvements and validation

- [x] T038 Add comprehensive error handling for agent operations
- [x] T039 Implement performance measurements and response time tracking
- [x] T040 Add comprehensive logging for debugging and monitoring
- [x] T041 Create PHR documentation for agent implementation phases
- [x] T042 Validate all acceptance criteria are met

## Dependencies

- **US1 (P1)**: Environment setup and client configuration (T009-T012) - Required by all other stories
- **US2 (P1)**: Retrieval logic integration (T013-T016) - Depends on Setup
- **US3 (P2)**: Tool decoration and registration (T017-T020) - Depends on US1, US2
- **US4 (P2)**: Agent definition (T021-T024) - Depends on US1, US3
- **US5 (P3)**: Runner implementation (T025-T028) - Depends on US3, US4

## Parallel Execution Opportunities

**Within US1**: T009-T012 can run in parallel for environment setup
**Within US2**: T013-T016 can run in parallel with retrieval validation
**Within US3**: T017-T020 can run in parallel for tool implementation
**Within Testing**: T029-T033 can run in parallel for different test scenarios

## Test Scenarios

- Agent successfully imports and calls the retrieval function
- Agent correctly routes through OpenRouter's API
- Final output includes book metadata (URLs/headings) provided by the tool
- Agent maintains a "stateless" architecture (does not rely on local session files)
- Tool calling works reliably with proper error handling
- Response time is optimized for real-time interaction
- Agent follows instructions to use retrieval tool for every query
- Agent states when information is missing from documentation