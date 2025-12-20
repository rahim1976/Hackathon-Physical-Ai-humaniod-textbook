# AI/Spec-Driven Book with Embedded RAG Chatbot - Frontend

This is the frontend component of an AI-powered book with an embedded Retrieval-Augmented Generation (RAG) chatbot, built using Spec-Driven Development methodology with the Spec-Kit Plus framework.

## Project Overview

This directory contains a Docusaurus-based book with integrated RAG chatbot that provides contextual answers based solely on the book's content. The system ensures no hallucinated responses by grounding all answers in the documented material.

## Architecture

- **Frontend**: Docusaurus-based documentation site deployed on GitHub Pages
- **Backend**: FastAPI service for handling chatbot requests (located in parent directory)
- **Vector Storage**: Qdrant Cloud for document embeddings
- **Database**: Neon Postgres for metadata and session storage
- **AI Integration**: OpenAI API for natural language processing

## Setup Instructions

1. Navigate to this directory: `cd frontendofbook`
2. Install dependencies: `npm install`
3. Set up environment variables (see `.env.example`)
4. Start the development server: `npm run dev`

## Development Workflow

This project follows a spec-driven development approach:
1. Define specifications in the parent directory's `specs/` directory
2. Generate implementation tasks using Spec-Kit Plus
3. Implement features following the specifications
4. Validate against original requirements

## Contributing

Please follow the established spec-driven workflow when contributing to this project. All changes must be reflected in the appropriate specification files.