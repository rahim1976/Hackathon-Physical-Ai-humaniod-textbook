# Quickstart: Embedding Pipeline Setup

## Overview
Quick setup guide for the embedding pipeline that extracts text from Docusaurus URLs, generates embeddings using Cohere, and stores them in Qdrant with validation and logging.

## Prerequisites
- Python 3.11 or higher
- Cohere API key
- Qdrant cluster URL and API key
- UV package manager installed

## Setup

### 1. Navigate to the backend directory
```bash
cd backend/embedding_pipeline
```

### 2. Initialize project with UV Package
```bash
# Create pyproject.toml file for UV Package management
uv init
```

### 3. Install dependencies
```bash
uv pip install cohere-client qdrant-client requests beautifulsoup4 python-dotenv
```

### 4. Create environment file
```bash
cp .env.example .env
```

Add your actual keys to the `.env` file:
```
COHERE_API_KEY=your_actual_cohere_api_key_here
QDRANT_URL=your_actual_qdrant_cluster_url_here
QDRANT_API_KEY=your_actual_qdrant_api_key_here
DOCUSAURUS_BASE_URL=https://hackathon-physical-ai-humaniod-textbook.vercel.app/
```

## Usage

### 1. Run the embedding pipeline in crawl mode
```bash
python main.py --mode crawl
```

### 2. Run the embedding pipeline in test mode for a single URL
```bash
python main.py --mode test --url "https://hackathon-physical-ai-humaniod-textbook.vercel.app/docs/intro"
```

### 3. The script will:
- Initialize Cohere and Qdrant clients
- Fetch all URLs from the deployed Docusaurus site
- Extract clean text content from each URL
- Chunk the content appropriately
- Generate embeddings using Cohere
- Create a "rag_embedding" collection in Qdrant
- Store embeddings with metadata in Qdrant
- Validate retrieval and log results

## Configuration
- Adjust the `DOCUSAURUS_BASE_URL` in `.env` to point to your Docusaurus deployment
- Modify chunk size with `--chunk-size` parameter if needed
- Adjust chunk overlap with `--chunk-overlap` parameter if needed
- Specify a custom collection name with `--collection` parameter

## Validation and Logging
- Retrieval validation is performed automatically
- Detailed logs are output to console
- Progress tracking shows pipeline execution status
- Error handling and retry mechanisms ensure robust operation

## Troubleshooting
- Ensure your Cohere and Qdrant API keys are valid
- Verify that the Qdrant URL is accessible
- Check that the Docusaurus site is publicly accessible
- Monitor API rate limits during processing
- Enable verbose logging with `--verbose` flag for detailed debugging