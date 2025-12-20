# AI/Spec-Driven Book with Embedded RAG Chatbot

This project implements an AI-powered book with an embedded Retrieval-Augmented Generation (RAG) chatbot, built using Spec-Driven Development methodology with the Spec-Kit Plus framework.

## Project Structure

The project is now organized with the frontend components in a dedicated directory:

```
.
├── frontendofbook/           # Docusaurus-based frontend
│   ├── docs/                # Documentation content
│   ├── src/                 # Source code
│   ├── docusaurus.config.js # Docusaurus configuration
│   ├── sidebars.js          # Navigation configuration
│   └── package.json         # Frontend dependencies and scripts
├── src/                     # Backend source code (moved from frontend)
├── specs/                   # Feature specifications
├── tests/                   # Test files
├── .specify/                # Spec-Kit Plus configuration
├── .github/                 # GitHub configuration
└── history/                 # Prompt history records
```

## Architecture

- **Frontend**: Docusaurus-based documentation site in `frontendofbook/` deployed on GitHub Pages
- **Backend**: FastAPI service for handling chatbot requests
- **Vector Storage**: Qdrant Cloud for document embeddings
- **Database**: Neon Postgres for metadata and session storage
- **AI Integration**: OpenAI API for natural language processing

## Setup Instructions

### Frontend Setup
1. Navigate to frontend directory: `cd frontendofbook`
2. Install dependencies: `npm install`
3. Start the development server: `npm run dev`

### Backend Setup
1. Install Python dependencies: `pip install -r requirements.txt`
2. Set up environment variables (see `.env.example`)
3. Start the backend: `python -m src.backend.main`

## Development Workflow

This project follows a spec-driven development approach:
1. Define specifications in the `specs/` directory
2. Generate implementation tasks using Spec-Kit Plus
3. Implement features following the specifications
4. Validate against original requirements

## Contributing

Please follow the established spec-driven workflow when contributing to this project. All changes must be reflected in the appropriate specification files.