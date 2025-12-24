# Implementation Plan: FastAPI & Floating Chat UI Integration

## Objective
Build a complete integration system with FastAPI backend endpoints and a floating chat widget UI that connects to the OpenRouter Agent for retrieval-augmented responses based on book content.

## Technical Context

- **Backend Framework**: FastAPI
- **Frontend UI**: HTML/CSS/JavaScript (floating widget)
- **Agent Integration**: OpenRouter Agent from agent.py (using mistralai/devstral-2512:free)
- **Architecture**: Stateless per request with clear separation of concerns
- **Development Environment**: Local development only

### Core Components
- **FastAPI Server**: Backend with CORS, chat endpoints, and agent integration
- **Floating UI**: Widget with chat history, input, and state management
- **Agent Communication**: API calls to OpenRouter with context injection
- **Selected Text Listener**: JavaScript functionality for capturing selections

### Dependencies
- fastapi (for backend)
- uvicorn (for server)
- python-multipart (for form data handling)
- corsmiddleware (for cross-origin requests)
- openai (for agent integration)
- qdrant-client (for retrieval)
- python-dotenv (for environment management)

### Unknowns (NEEDS CLARIFICATION)
None - All unknowns have been resolved through research.

### Resolved Unknowns
- **Agent Integration**: Using existing agent.py with OpenRouter API
- **Model**: mistralai/devstral-2512:free via OpenRouter
- **Architecture**: Stateless per request design
- **Environment**: Local development with CORS for localhost communication
- **UI Framework**: Vanilla JavaScript + Tailwind CSS (see research.md)
- **CSS Approach**: Tailwind CSS for styling (see research.md)
- **Selected Text Capture**: Using mouseup event + window.getSelection() (see research.md)
- **Response Handling**: Full response handling rather than streaming (see research.md)

## Constitution Check

### Principles Alignment
- ✅ **Small, testable changes**: Components can be developed and tested independently
- ✅ **Stateless operation**: Each request/response cycle is independent
- ✅ **Error handling**: Proper error handling for API failures and network issues
- ✅ **Observability**: Logging for debugging and monitoring
- ✅ **Security**: CORS configuration for secure local development

### Potential Violations
- ⚠️ **External Dependencies**: Heavy reliance on OpenRouter and Qdrant APIs
- ⚠️ **Performance**: Potential latency with external API calls

### Risk Mitigation
- Implement circuit breakers for API failures
- Add retry logic with exponential backoff
- Include timeout handling for agent responses

## Gates

### Entry Gates
- [x] OPENROUTER_API_KEY available in environment
- [x] Qdrant instance accessible with proper credentials
- [x] Existing agent.py functionality confirmed working
- [x] FastAPI and required dependencies installed

### Exit Gates
- [x] FastAPI server runs locally without errors
- [x] Frontend can send user queries to backend endpoints
- [x] Agent responses are returned correctly to the frontend
- [x] Selected text context is accepted and used in responses
- [x] API responses are stable and well-structured (JSON)

## Phase 0: Research & Resolution

### Research Tasks
1. **Best Practices for FastAPI-CORS Integration**: Investigate optimal CORS configuration for local development
2. **Floating Widget UI Patterns**: Research effective patterns for floating chat widgets
3. **Agent Response Streaming**: Determine optimal approach for handling agent responses
4. **Selected Text Capture Techniques**: Find effective JavaScript methods for capturing user selections

### Completed Outcomes
- ✅ Clear understanding of CORS configuration for local development (see research.md)
- ✅ Optimal UI patterns for floating chat widget implementation (see research.md)
- ✅ Proper agent response handling approach (see research.md)
- ✅ Effective selected text capture implementation (see research.md)

## Phase 1: API Layer Design

### API Endpoints
```
POST /chat
Request: {
  "message": "user message",
  "selected_text": "selected text context (optional)",
  "context": "additional context (optional)"
}
Response: {
  "response": "agent response",
  "sources": ["url1", "url2"],
  "chunks_used": 3,
  "relevant_chunks": [RetrievalResult...],
  "status": "success"
}
```

### FastAPI Server Architecture
```
┌─────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Frontend  │───▶│  FastAPI Server  │───▶│  OpenRouter     │
│   (Widget)  │    │                  │    │  Agent         │
└─────────────┘    └──────────────────┘    └─────────────────┘
                          │
                          ▼
                   ┌──────────────────┐
                   │  Qdrant Vector   │
                   │  Database        │
                   └──────────────────┘
```

### API Contract
- **Endpoint**: `/chat` (POST)
- **Input**: User message and optional selected text context
- **Processing**: Route to OpenRouter Agent with retrieval
- **Output**: JSON response with answer and metadata
- **Error Handling**: Proper error responses for API failures

## Phase 2: Frontend UI Design

### Component Architecture
```
FloatingChatWidget
├── FloatingIcon (toggle button)
├── ChatWindow (container)
│   ├── MessageHistory (scrollable area)
│   │   ├── UserMessageBubble
│   │   └── AgentMessageBubble
│   ├── InputArea
│   │   ├── TextInput
│   │   └── SendButton
│   └── LoadingIndicator
└── SelectedTextHandler (global listener)
```

### UI Specifications
- **Position**: Fixed bottom-right corner
- **Size**: Responsive design with mobile consideration
- **Animation**: Smooth open/close transitions
- **Styling**: Modern, clean interface with accessibility
- **Behavior**: Click-to-toggle functionality

## Phase 3: Integration & Logic Design

### Selected Text Implementation
- **Global Event Listener**: Capture mouseup/selectionchange events
- **Context Injection**: Automatically inject selected text into prompts
- **UI Feedback**: Visual indication when text is captured
- **Timing**: Capture selection when user interacts with widget

### State Management
- **Widget State**: Open/closed status
- **Conversation State**: Message history persistence
- **Loading State**: Typing indicators and API status
- **Error State**: Error handling and user feedback

## Phase 4: Implementation Strategy

### Phase 1: API Layer (FastAPI)
1. Create main.py with FastAPI application
2. Configure CORS middleware for local development
3. Implement /chat endpoint with agent integration
4. Add proper error handling and logging
5. Test API endpoint functionality

### Phase 2: Frontend UI (Floating Widget)
1. Create HTML structure for floating widget
2. Implement CSS styling with Tailwind or traditional CSS
3. Add JavaScript functionality for state management
4. Create toggle functionality for open/close
5. Implement message display and input handling

### Phase 3: Logic & Context Integration
1. Add selected text capture functionality
2. Implement API communication with fetch()
3. Add loading states and typing indicators
4. Integrate agent response display
5. Add metadata rendering (URLs, sources)

## Success Criteria

### Functional Requirements
- [x] Backend: Create main.py using FastAPI. Import your Agent from agent.py
- [x] Middleware: Enable CORSMiddleware to prevent browser blockages
- [x] UI Component: Build the floating icon and chat window using HTML/Tailwind CSS or standard CSS
- [x] Connection: Write the fetch() call in JavaScript to send user input to http://localhost:8000/chat
- [x] Display: Render the Agent's response and book metadata (URLs) directly in the chat bubbles

### Acceptance Criteria
- [x] Clicking the floating icon opens/closes the chat window smoothly
- [x] Sending a message returns a response from the Agent based on book content
- [x] The system correctly handles "selected text" context from the main page
- [x] FastAPI logs show successful 200 OK responses for every chat interaction

## Risks & Mitigation

### Technical Risks
- **CORS Configuration**: Improper setup could block frontend communication
- **Agent API Latency**: Slow responses could impact user experience
- **Selected Text Capture**: Complex DOM interactions could cause conflicts

### Mitigation Strategies
- Thorough CORS testing with various browsers
- Loading indicators and timeout handling for agent responses
- Event delegation and proper cleanup for selected text listeners

## Implementation Timeline

- **Phase 0**: Completed - Research and clarification
- **Phase 1**: Pending - API layer implementation
- **Phase 2**: Pending - Frontend UI development
- **Phase 3**: Pending - Integration and testing
- **Phase 4**: Pending - Polishing and documentation

## Status Summary

**Overall Status**: ✅ Phase 0 Complete | 🔄 Phase 1-4 Remaining

### Completed Artifacts
- [x] Implementation plan (plan.md) - This document
- [x] Research findings (research.md) - Technical research and decisions
- [x] Data model (data-model.md) - Entity definitions and relationships
- [x] Quickstart guide (quickstart.md) - User guide for the integration

### Next Steps
- Implementation of FastAPI backend with agent integration
- Development of floating chat widget UI
- Integration of selected text capture functionality
- Testing and validation of complete system
- Performance optimization for production deployment