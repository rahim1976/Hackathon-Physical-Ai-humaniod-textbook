# Data Model: RAG Retrieval Testing

**Feature**: RAG Retrieval Testing
**Created**: 2025-12-21

## Entities

### QueryRequest
- **Fields**:
  - query: string (the text query to search for)
  - topK: integer (optional, number of results to return, default 5)
  - filters: object (optional, additional filters for search)

### RetrievedChunk
- **Fields**:
  - id: string (unique identifier from Qdrant)
  - content: string (the actual text content retrieved)
  - url: string (source URL of the content)
  - position: integer (position of the chunk in the original document)
  - score: number (relevance score from similarity search)
  - createdAt: number (timestamp when the chunk was stored)

### RetrievalResponse
- **Fields**:
  - chunks: array of RetrievedChunk (the retrieved text chunks)
  - query: string (the original query text)
  - totalResults: integer (total number of results found)
  - processingTime: number (time taken to process the query in milliseconds)

### ValidationError
- **Fields**:
  - expectedContent: string (what content was expected)
  - actualContent: string (what content was retrieved)
  - similarityScore: number (measure of how similar the content is)
  - metadataMismatch: object (details about any metadata mismatches)

## Relationships

- QueryRequest is processed to generate RetrievalResponse
- RetrievalResponse contains multiple RetrievedChunk instances
- RetrievedChunk may be associated with ValidationError if content validation fails

## Validation Rules

- Query text must not be empty
- topK value must be between 1 and 100
- Retrieved content similarity must be above 95% threshold
- Metadata (URL, position) must match original stored values
- Response must be in valid JSON format