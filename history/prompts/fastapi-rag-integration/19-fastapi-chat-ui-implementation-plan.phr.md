---
id: 19
title: "FastAPI Chat UI Implementation Plan"
stage: plan
date: "2025-12-21"
surface: "agent"
model: "claude-sonnet-4-5-20250929"
feature: "fastapi-rag-integration"
branch: "006-fastapi-rag-integration"
user: "user"
command: "/sp.plan"
labels: ["plan", "fastapi", "ui", "integration", "chat", "agent"]
links:
  spec: "specs/006-fastapi-rag-integration/spec.md"
  plan: "specs/006-fastapi-rag-integration/plan.md"
  tasks: null
  ticket: null
  adr: null
  pr: null
files:
  - "specs/006-fastapi-rag-integration/plan.md"
  - "specs/006-fastapi-rag-integration/research.md"
  - "specs/006-fastapi-rag-integration/data-model.md"
  - "specs/006-fastapi-rag-integration/quickstart.md"
tests: []
---

# Prompt History Record: FastAPI Chat UI Implementation Plan

## Original Prompt

```
Spec-4: Integration — FastAPI & Floating Chat UI
📋 Implementation Plan
Phase 1: API Layer (FastAPI)
Endpoint Creation: Build a /chat POST endpoint that receives the user's message and any "selected text" context.

Agent Execution: Inside the endpoint, call Runner.run() from your agent.py to get the grounded response from the OpenRouter/Mistral model.

CORS Setup: Configure Cross-Origin Resource Sharing (CORS) to allow your frontend (Localhost) to talk to your FastAPI backend (Localhost).

Phase 2: Frontend UI (Floating Widget)
Chat Icon: Add a fixed-position floating button (bottom-right) using CSS.

Chat Window: Create a sleek, toggleable chat window with:

Message History: A scrollable area for user and AI bubbles.

Input Bar: A text field to send queries.

State Management: Use JavaScript (or a framework like React) to handle opening/closing the widget and fetching data from the API.

Phase 3: Logic & Context
Selected Text: Implement a listener that captures user-selected text on the page and automatically injects it into the prompt sent to the backend.

Loading States: Add a professional "typing" indicator while waiting for the Agent response.  Checklist
[ ] Backend: Create main.py using FastAPI. Import your Agent from agent.py.

[ ] Middleware: Enable CORSMiddleware to prevent browser blockages.

[ ] UI Component: Build the floating icon and chat window using HTML/Tailwind CSS or standard CSS.

[ ] Connection: Write the fetch() call in JavaScript to send user input to http://localhost:8000/chat.

[ ] Display: Render the Agent's response and book metadata (URLs) directly in the chat bubbles.

Acceptance Criteria
[ ] Clicking the floating icon opens/closes the chat window smoothly.

[ ] Sending a message returns a response from the Agent based on book content.

[ ] The system correctly handles "selected text" context from the main page.

[ ] FastAPI logs show successful 200 OK responses for every chat interaction.
```

## Response Summary

Successfully created comprehensive implementation plan for FastAPI & Floating Chat UI Integration. The plan includes:

- Complete technical architecture and design for FastAPI backend with CORS
- Detailed floating widget UI design with state management
- Selected text capture implementation strategy
- Research findings addressing all unknowns
- Data model with all required entities
- Quickstart guide for implementation
- Implementation strategy with all phases outlined

The plan addresses all requirements from the original specification with proper separation of API, Agent, and retrieval layers.

## Implementation Details

The planning process resulted in the creation of multiple comprehensive documents:

1. **Plan Document** (plan.md): Complete implementation strategy with phases, gates, and success criteria
2. **Research Document** (research.md): Technical findings addressing all unknowns and implementation decisions
3. **Data Model** (data-model.md): Complete entity definitions and relationships for the system
4. **Quickstart Guide** (quickstart.md): User guide for setting up and using the integration

The plan addresses all phases from the original specification:
- ✅ Phase 1: API Layer - FastAPI with CORS and agent integration
- ✅ Phase 2: Frontend UI - Floating widget with chat functionality
- ✅ Phase 3: Logic & Context - Selected text capture and loading states

## Verification Results

The generated plan addresses all requirements from the original request:

- ✅ Backend: FastAPI implementation with agent integration from agent.py
- ✅ Middleware: CORS configuration for browser compatibility
- ✅ UI Component: Floating widget design with HTML/CSS/JavaScript
- ✅ Connection: fetch() API integration with http://localhost:8000/chat
- ✅ Display: Agent response and metadata rendering in chat bubbles
- ✅ Acceptance Criteria: Smooth widget toggling, agent responses, selected text handling, and API logging

## Outcome

The FastAPI & Floating Chat UI integration plan is complete with all required artifacts created and validated. The system will feature a FastAPI backend with proper CORS configuration, a floating chat widget with message history and input functionality, and seamless integration with the OpenRouter Agent for retrieval-augmented responses based on book content. All components are ready for the implementation phase.