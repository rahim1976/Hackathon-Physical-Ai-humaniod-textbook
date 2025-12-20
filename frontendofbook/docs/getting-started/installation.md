---
sidebar_position: 1
---

# Installation

This guide will walk you through setting up the AI/Spec-Driven Book with Embedded RAG Chatbot project on your local machine.

## Prerequisites

Before you begin, ensure you have the following installed:

- Node.js (version 18 or higher)
- npm or yarn package manager
- Python 3.8+ (for backend services)
- Git

## Quick Start

The fastest way to get started is to clone the repository and install dependencies:

```bash
# Clone the repository
git clone <repository-url>
cd ai-spec-driven-book

# Install frontend dependencies
npm install
```

## Environment Setup

Create a `.env` file by copying the example:

```bash
cp .env.example .env
```

Then fill in the required environment variables:

- `OPENAI_API_KEY`: Your OpenAI API key
- `QDRANT_URL`: Your Qdrant Cloud instance URL
- `QDRANT_API_KEY`: Your Qdrant API key
- `DATABASE_URL`: Connection string for Neon Postgres

## Backend Dependencies

Install Python dependencies for the backend services:

```bash
pip install fastapi uvicorn openai qdrant-client psycopg2-binary python-dotenv
```

## Running the Development Server

Start the Docusaurus development server:

```bash
npm run dev
```

This will start the documentation site on `http://localhost:3000`.

## Next Steps

Once you have the basic installation complete, proceed to [Configuration](./configuration.md) to set up your AI services and customize the system for your content.