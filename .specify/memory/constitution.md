<!-- Sync Impact Report:
Version change: None → 1.0.0
List of modified principles:
- [PRINCIPLE_1_NAME] → Spec-first workflow using Spec-Kit Plus
- [PRINCIPLE_2_NAME] → Technical accuracy from official sources
- [PRINCIPLE_3_NAME] → Clear, developer-focused writing
- [PRINCIPLE_4_NAME] → Reproducible setup and deployment
Added sections: Key Standards, Constraints
Removed sections: None
Templates requiring updates:
- .specify/templates/plan-template.md: ✅ updated
- .specify/templates/spec-template.md: ✅ updated
- .specify/templates/tasks-template.md: ✅ updated
- .specify/templates/commands/*.md: ✅ updated
- README.md: ⚠ pending
Follow-up TODOs: TODO(README.md): Update references to principles changed.
-->
# AI/Spec-Driven Book with Embedded RAG Chatbot Constitution

## Core Principles

### Spec-first workflow using Spec-Kit Plus
Every feature and implementation MUST begin with a clear, approved specification generated and managed using Spec-Kit Plus.

### Technical accuracy from official sources
All technical information, documentation, and code MUST be grounded in official, authoritative sources. No assumptions or unsupported claims are permitted.

### Clear, developer-focused writing
All documentation, code comments, and explanatory text MUST be clear, concise, and written with a focus on developer usability and understanding.

### Reproducible setup and deployment
The project setup, development environment, and deployment process MUST be fully reproducible from source, with comprehensive documentation.

## Key Standards

Book written with Docusaurus and deployed on GitHub Pages. RAG chatbot MUST be grounded only in book content or user-selected text. Stack: OpenAI Agents/Chatkit, FastAPI, Neon Postgres, Qdrant Cloud. All code MUST be runnable and well-documented.

## Constraints

GitHub-based source control is mandatory. The RAG chatbot MUST NOT produce hallucinated responses. The entire project MUST demonstrate end-to-end reproducibility.

## Governance
All implementations MUST result in a live book on GitHub Pages. The embedded RAG chatbot MUST be fully functional. All specifications MUST be implemented via Spec-Kit Plus. Amendments to this constitution require documentation, approval, and a migration plan. All PRs/reviews must verify compliance. Complexity must be justified.

**Version**: 1.0.0 | **Ratified**: 2025-12-18 | **Last Amended**: 2025-12-18
