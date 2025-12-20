---
sidebar_position: 2
---

# Configuration

Learn how to configure the AI/Spec-Driven Book with Embedded RAG Chatbot for your specific needs.

## AI Service Configuration

### OpenAI Setup

The system uses OpenAI's API for natural language processing. Configure your API key in the `.env` file:

```env
OPENAI_API_KEY=your_openai_api_key_here
CHATBOT_MODEL=gpt-4-turbo  # Recommended model
MAX_TOKENS=1000
TEMPERATURE=0.7
```

### Vector Database Configuration

#### Qdrant Cloud Setup

For production deployments, we recommend using Qdrant Cloud:

1. Sign up at [qdrant.tech](https://qdrant.tech)
2. Create a new cluster
3. Get your cluster URL and API key
4. Add to your `.env` file:

```env
QDRANT_URL=your-qdrant-cluster-url
QDRANT_API_KEY=your-qdrant-api-key
```

For local development, you can run Qdrant locally:

```bash
docker run -d --name qdrant-container -p 6333:6333 qdrant/qdrant
```

And use:

```env
QDRANT_URL=http://localhost:6333
```

### Database Configuration

#### Neon Postgres Setup

Neon Postgres provides serverless PostgreSQL with auto-scaling:

1. Sign up at [neon.tech](https://neon.tech)
2. Create a new project
3. Copy the connection string to your `.env` file:

```env
DATABASE_URL=your-neon-postgres-connection-string
```

## Content Configuration

### Adding Documentation

Place your documentation files in the `docs/` directory. The system will automatically index these for the RAG chatbot.

### Customizing the Book

Modify `docusaurus.config.js` to customize navigation, branding, and other site-wide settings:

```javascript
module.exports = {
  title: 'Your Book Title',
  tagline: 'Your book tagline',
  // ... other configuration
};
```

## RAG System Configuration

### Indexing Settings

Control how documents are chunked and indexed in the RAG system:

```env
CHUNK_SIZE=1000       # Number of characters per chunk
CHUNK_OVERLAP=100     # Overlap between chunks
VECTOR_DIMENSION=1536 # Dimension of embedding vectors
```

### Search Parameters

Fine-tune the retrieval process:

```env
SEARCH_TOP_K=5        # Number of documents to retrieve
SEARCH_THRESHOLD=0.7  # Similarity threshold for relevance
```

## Next Steps

With your system configured, learn about the [AI Integration](../ai-integration/overview.md) and how the RAG system works.