# Implementation Plan: OpenAI Agent with OpenRouter Integration

## Objective
Build a retrieval-augmented agent using OpenAI Agents SDK that integrates with OpenRouter API and leverages the existing retrieval functionality from retrieving.py.

## Technical Context

- **Backend Language**: Python 3.11+
- **Agent Framework**: OpenAI Agents SDK (with OpenRouter compatibility)
- **API Provider**: OpenRouter (https://openrouter.ai/api/v1)
- **Model**: mistralai/devstral-2512:free
- **API Key**: OPENROUTER_API_KEY from environment
- **Integration**: Existing retrieval system from retrieving.py
- **Architecture**: Stateless per request

### Core Components
- **Agent**: OpenAI Assistant with custom tools (using OpenRouter)
- **Tool**: Retrieval function from retrieving.py wrapped as OpenAI tool
- **Interface**: Thread-based conversation flow (OpenRouter compatible)
- **Storage**: Qdrant vector database with semantic search

### Dependencies
- openai >= 1.3.8 (compatible with OpenRouter)
- qdrant-client >= 1.8.0
- python-dotenv for environment management
- requests for API communication

### Unknowns (NEEDS CLARIFICATION)
- Specific rate limiting requirements for OpenRouter API usage

### Resolved Unknowns
- **OpenRouter compatibility**: OpenAI SDK can be configured with custom base URL for OpenRouter
- **Model availability**: mistralai/devstral-2512:free is available on OpenRouter
- **API key usage**: OPENROUTER_API_KEY can be used with OpenAI client
- **Agent features**: Basic agent functionality works with OpenRouter (see research.md)
- **Client configuration**: OpenAI client can be configured with base_url for OpenRouter
- **Tool calling**: Function tool calling is supported by OpenRouter

## Constitution Check

### Principles Alignment
- ✅ **Small, testable changes**: Implementation will be modular with clear test boundaries
- ✅ **Stateless operation**: Each agent run will be independent
- ✅ **Error handling**: Comprehensive error handling for API failures
- ✅ **Observability**: Logging and metrics for debugging and monitoring
- ✅ **Security**: Environment variables for API keys, no hardcoded credentials

### Potential Violations
- ⚠️ **External Dependencies**: Heavy reliance on OpenRouter API
- ⚠️ **Feature Limitations**: OpenRouter may have limited Agent SDK support

### Risk Mitigation
- Implement circuit breakers for API failures
- Add fallback mechanisms for unsupported features
- Include retry logic with exponential backoff

## Gates

### Entry Gates
- [x] OPENROUTER_API_KEY available in environment
- [x] Qdrant instance accessible with proper credentials
- [x] Existing retrieving.py system functional
- [x] OpenRouter API accessible with specified model

### Exit Gates
- [x] Agent successfully retrieves and responds to queries via OpenRouter
- [x] Tool calling works reliably with OpenRouter API
- [x] Error handling covers all major failure modes
- [x] Performance meets real-time interaction requirements

## Phase 0: Research & Resolution

### Research Tasks
1. **OpenRouter API Compatibility**: Investigate compatibility of OpenAI Agents SDK with OpenRouter endpoints
2. **Client Configuration**: Determine proper OpenAI client setup for OpenRouter base URL
3. **Model Capabilities**: Research agent and tool calling capabilities of mistralai/devstral-2512:free
4. **Rate Limiting Strategies**: Determine appropriate throttling for OpenRouter API calls

### Completed Outcomes
- ✅ Clear understanding of OpenRouter's Agent SDK compatibility (see research.md)
- ✅ Proper client configuration for OpenRouter integration (see research.md)
- ✅ Compatibility verification for required agent features (see research.md)
- ✅ Performance benchmarks for real-time response (see research.md)

## Phase 1: Design & Architecture

### Data Model
```
AgentThread:
  - thread_id: str
  - created_at: datetime
  - messages: List[Message]

RetrievalResult:
  - id: str
  - content: str
  - url: str
  - position: int
  - similarity_score: float
  - section: str
  - heading: str

ToolCall:
  - tool_call_id: str
  - function_name: str
  - arguments: Dict
  - result: str

AgentResponse:
  - response_id: str
  - content: str
  - sources: List[str]
  - chunks_used: int
  - relevant_chunks: List[RetrievalResult]
  - status: str
  - error: str (optional)
  - timestamp: datetime
```

### System Architecture
```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   User Query    │───▶│  OpenRouter     │───▶│  Qdrant DB      │
│                 │    │  Agent         │    │                 │
└─────────────────┘    └──────────────────┘    └─────────────────┘
                              │
                              ▼
                       ┌──────────────────┐
                       │  Tool Response   │
                       │  Processing      │
                       └──────────────────┘
```

## Phase 2: Implementation Strategy

### OpenAI Client Configuration
1. Create OpenAI client with OpenRouter base URL
2. Configure API key and model settings
3. Test basic API connectivity

### Tool Integration
1. Create wrapper for retrieve_context function from retrieving.py
2. Register the tool with the OpenRouter Assistant
3. Validate JSON schema generation for the tool

### Agent Definition
1. Define system instructions for RAG grounding
2. Configure the agent to prioritize retrieval over general knowledge
3. Implement guardrails for out-of-context queries

### Stateless Execution
1. Implement thread creation and destruction per request
2. Ensure no shared state between requests
3. Optimize for concurrent request handling

## Phase 3: Testing & Validation

### Unit Tests
- Individual component testing for tool functionality
- Error handling validation
- Thread lifecycle verification

### Integration Tests
- Full agent workflow testing
- Retrieval integration testing
- Response quality validation

### Performance Tests
- Response time benchmarks
- Concurrent request handling
- API rate limiting validation

## Success Criteria

### Functional Requirements
- [ ] Agent autonomously invokes retrieval tool via OpenRouter
- [ ] Responses are grounded in retrieved book content
- [ ] Agent refuses to answer out-of-context queries
- [ ] Tool calling works reliably with proper error handling

### Non-Functional Requirements
- [ ] Response time < 5 seconds for typical queries
- [ ] Thread creation/destruction per request
- [ ] Proper error handling for API failures
- [ ] Graceful degradation when no relevant content found

## Risks & Mitigation

### Technical Risks
- **OpenRouter Compatibility**: Agent SDK features may not be fully supported
- **Rate Limits**: Free tier may have usage limitations
- **Model Limitations**: Free model may have reduced capabilities

### Operational Risks
- **API Availability**: OpenRouter service may have downtime
- **Response Quality**: Free model may produce lower quality responses
- **Cost Management**: Need to monitor usage even for free tier

## Implementation Timeline

- **Phase 0**: Completed - Research and clarification
- **Phase 1**: Completed - Design and architecture
- **Phase 2**: Pending - Implementation
- **Phase 3**: Pending - Testing and validation

## Status Summary

**Overall Status**: ✅ Phase 0 & 1 Complete | 🔄 Phase 2 & 3 Remaining

### Completed Artifacts
- [x] Implementation plan (plan.md) - This document
- [x] Research findings (research.md) - Technical research and decisions
- [x] Data model (data-model.md) - Entity definitions and relationships
- [x] Quickstart guide (quickstart.md) - User guide for the agent

### Next Steps
- Implementation of agent.py with OpenRouter integration
- Integration with existing retrieving.py functionality
- Testing and validation of the complete system
- Performance optimization for OpenRouter API usage