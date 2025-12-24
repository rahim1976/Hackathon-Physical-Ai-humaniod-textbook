# Data Model: FastAPI & Floating Chat UI Integration

## Core Entities

### APIRequest
**Description**: Represents a request sent from the frontend to the FastAPI backend

**Fields**:
- `message` (str): The user's message/query (required)
- `selected_text` (str): Selected text context (optional, default: "")
- `context` (str): Additional context information (optional, default: "")
- `user_id` (str): User identifier (optional for anonymous usage)
- `timestamp` (datetime): When the request was made (required)

**Validation Rules**:
- `message` must be 1-1000 characters
- `selected_text` can be empty or up to 5000 characters
- `context` must be 0-10000 characters
- `timestamp` must be in ISO 8601 format

### APIResponse
**Description**: Represents the response from the FastAPI backend to the frontend

**Fields**:
- `response` (str): The agent's response to the user query (required)
- `sources` (List[str]): List of source URLs referenced (required)
- `chunks_used` (int): Number of retrieval results used (required)
- `relevant_chunks` (List[RetrievalResult]): Detailed information about chunks used (optional)
- `status` (str): Response status (success, error, timeout) (required)
- `request_id` (str): Unique identifier for the request (required)
- `timestamp` (datetime): When response was generated (required)
- `error` (str): Error message if status is error (optional)

**Validation Rules**:
- `status` must be one of: "success", "error", "timeout"
- `chunks_used` must match length of `relevant_chunks` if provided
- `request_id` must be a valid UUID format
- `response` must not exceed 10000 characters

### RetrievalResult
**Description**: Represents a single retrieved chunk from Qdrant (used in APIResponse)

**Fields**:
- `id` (str): Unique identifier for the chunk in Qdrant (required)
- `content` (str): The retrieved text content (required)
- `url` (str): Source URL for the content (required)
- `position` (int): Position in the original document (optional, default: 0)
- `similarity_score` (float): Semantic similarity score (0.0-1.0) (required)
- `section` (str): Section name if available (optional)
- `heading` (str): Heading name if available (optional)
- `created_at` (datetime): When the chunk was indexed (optional)

**Validation Rules**:
- `similarity_score` must be between 0.0 and 1.0
- `content` must not be empty
- `url` must be a valid URL format if provided

### Message
**Description**: Represents a single message in the chat history

**Fields**:
- `id` (str): Unique identifier for the message (required)
- `role` (str): Role of the message sender (user, assistant) (required)
- `content` (str): The message content (required)
- `timestamp` (datetime): When the message was created (required)
- `sources` (List[str]): Source URLs associated with the message (optional)

**Validation Rules**:
- `role` must be one of: "user", "assistant"
- `content` must not exceed 10000 characters
- `timestamp` must be in ISO 8601 format

### ChatSession
**Description**: Represents a chat session in the frontend UI

**Fields**:
- `session_id` (str): Unique identifier for the session (required)
- `messages` (List[Message]): List of messages in the conversation (required)
- `created_at` (datetime): When the session was created (required)
- `last_active` (datetime): Last activity timestamp (required)

**Validation Rules**:
- `session_id` must be a valid UUID format
- `messages` must contain at least one message
- `last_active` must be equal to or later than `created_at`

## State Transitions

### ChatSession State Transitions
```
CREATED → ACTIVE → INACTIVE
     ↓         ↓         ↓
   CLOSED ←────────────────
```

- **CREATED**: Session initialized, waiting for first message
- **ACTIVE**: Session processing messages
- **INACTIVE**: Session idle for extended period
- **CLOSED**: Session terminated (cleanup)

### APIResponse State Transitions
```
PENDING → PROCESSING → COMPLETED
     ↓         ↓           ↓
   TIMEOUT ←───────── ERROR
```

- **PENDING**: Request received, waiting to process
- **PROCESSING**: Agent is working on the request
- **COMPLETED**: Response successfully generated
- **TIMEOUT**: Request exceeded maximum processing time
- **ERROR**: Error occurred during processing

## API Data Structures

### Request Models

#### ChatRequest
```json
{
  "message": "user query text",
  "selected_text": "text selected by user (optional)",
  "context": "additional context (optional)"
}
```

**Validation**:
- `message` must be 1-1000 characters
- `selected_text` can be empty or up to 5000 characters
- `context` can be empty or up to 10000 characters

#### ChatResponse
```json
{
  "response": "agent response text",
  "sources": ["url1", "url2"],
  "chunks_used": 3,
  "relevant_chunks": [RetrievalResult...],
  "status": "success",
  "request_id": "unique_request_id",
  "timestamp": "2023-10-20T10:00:00Z"
}
```

**Validation**:
- `status` must be one of: "success", "error", "timeout"
- `chunks_used` must match length of `relevant_chunks` if provided
- `request_id` must be a valid UUID format
- `timestamp` must be in ISO 8601 format

## Database Schema Considerations

### Indexing Strategy
- `APIRequest.timestamp`: For time-based queries
- `RetrievalResult.similarity_score`: For relevance-based queries
- `RetrievalResult.url`: For source-based queries
- `Message.timestamp`: For chronological ordering

### Performance Considerations
- Store frequently accessed fields in memory when possible
- Implement caching for common retrieval results
- Use pagination for large message histories
- Implement proper connection pooling for Qdrant

## Security Considerations

### Data Validation
- Sanitize all user inputs before processing
- Validate URLs and other external references
- Implement size limits on content fields
- Use parameterized queries where applicable

### Privacy Protection
- Do not store sensitive user information unnecessarily
- Implement proper access controls for session data
- Encrypt sensitive data at rest
- Log only non-sensitive information for debugging