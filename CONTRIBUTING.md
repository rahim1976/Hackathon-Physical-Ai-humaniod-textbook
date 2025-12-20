# Contributing to AI/Spec-Driven Book with Embedded RAG Chatbot

Thank you for your interest in contributing to this project! This document outlines the process and guidelines for contributing.

## Project Overview

This project implements an AI-powered book with an embedded Retrieval-Augmented Generation (RAG) chatbot, built using Spec-Driven Development methodology with the Spec-Kit Plus framework.

## Development Workflow

This project follows the Spec-Driven Development (SDD) methodology:

1. **Specification** (`/sp.specify`): Define what needs to be built
2. **Planning** (`/sp.plan`): Design the implementation approach
3. **Tasking** (`/sp.tasks`): Break down work into testable tasks
4. **Implementation** (`/sp.implement`): Build the solution
5. **Review & Iterate**: Assess and refine

## Setting Up Your Development Environment

1. Clone the repository:
   ```bash
   git clone <repository-url>
   cd ai-spec-driven-book
   ```

2. Install Node.js dependencies:
   ```bash
   npm install
   ```

3. Install Python dependencies:
   ```bash
   pip install -r requirements.txt
   ```

4. Set up environment variables:
   ```bash
   cp .env.example .env
   # Edit .env with your actual values
   ```

5. For local development, you can run:
   ```bash
   # Start the Docusaurus documentation site
   npm run dev

   # Start the backend service in another terminal
   python -m src.backend.main --reload
   ```

## Running Tests

To run all tests:
```bash
python scripts/run_tests.py all
```

To run specific test types:
```bash
python scripts/run_tests.py unit      # Unit tests only
python scripts/run_tests.py end-to-end  # End-to-end tests only
```

To run linting checks:
```bash
python scripts/run_tests.py --lint-only
```

## Code Structure

```
├── docs/                 # Documentation (processed by Docusaurus)
├── src/                  # Source code
│   ├── api/             # API endpoints
│   ├── chatbot/         # RAG chatbot logic
│   └── backend/         # Backend services
├── specs/               # Feature specifications and plans
├── tests/               # Test files
├── scripts/             # Utility scripts
└── .specify/            # Spec-Kit Plus configuration
```

## Pull Request Process

1. Create a feature branch from the main branch
2. Follow the established coding standards
3. Add tests for new functionality
4. Update documentation as needed
5. Ensure all tests pass
6. Submit a pull request with a clear description

## Architecture Decision Records (ADRs)

Significant architectural decisions should be documented as ADRs. To create a new ADR:
```bash
/sp.adr "Title of the decision"
```

## Spec-Driven Development Commands

The project includes several command-line tools:

- `/sp.specify`: Create feature specifications
- `/sp.plan`: Generate implementation plans
- `/sp.tasks`: Create implementation tasks
- `/sp.implement`: Execute implementation tasks
- `/sp.adr`: Create architecture decision records

## Quality Standards

- All code must pass linting and formatting checks
- New features must include appropriate tests
- Documentation must be updated for user-facing changes
- Follow the existing code style and patterns
- Ensure responses are grounded in documentation to prevent hallucinations

## Getting Help

- Check the existing documentation in the `docs/` directory
- Review the issue tracker for similar problems
- Ask questions in pull requests or issues

## Code of Conduct

Please follow the project's code of conduct in all interactions.