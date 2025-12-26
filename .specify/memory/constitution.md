<!-- Sync Impact Report
Version change: 1.0.0 → 1.1.0
Modified principles:
  - I. Accuracy → I. Accuracy
  - II. Clarity → II. Privacy
  - III. Practicality → III. Performance
  - IV. Reproducibility → IV. Free Tier Compliance
  - V. Consistency → V. Citation
Added sections:
  - Architecture section
  - Data Pipeline section
  - Backend section
  - Frontend section
Removed sections: Original book creation principles
Templates requiring updates:
  - .specify/templates/plan-template.md ✅ updated
  - .specify/templates/spec-template.md ✅ updated
  - .specify/templates/tasks-template.md ✅ updated
Follow-up TODOs: none
-->
# RAG Chatbot for Physical AI & Humanoid Robotics Textbook Constitution

## Core Principles

### I. Accuracy
Answers only from book content, no hallucinations

### II. Privacy
No user data storage, minimal tracking

### III. Performance
< 3 second response time

### IV. Free Tier Compliance
No paid APIs, respect free tier limits

### V. Citation
Cite chapters/sections in all responses

## Architecture

1. Data Pipeline (Qdrant MCP):
   - Source: Docusaurus MDX files
   - Chunking: 500 tokens/chunk, 50-token overlap
   - Embeddings: Gemini text-embedding-004 (free)
   - Storage: Qdrant Cloud free tier (1GB)
   - Metadata: chapter, section, page URL

2. Backend (FastAPI):
   - Framework: FastAPI + async
   - AI: Gemini 1.5 Flash (free: 15 RPM, 1500 RPD)
   - Vector DB: Qdrant Cloud
   - ChatKit Python patterns
   - Endpoints:
     * POST /api/chat/query (general questions)
     * POST /api/chat/query-selection (selected text)
     * GET /api/health
   - Rate limit: 10 req/min/user

3. Frontend (React Widget):
   - ChatKit.js patterns for Docusaurus
   - Features: floating button, chat window, typing indicators
   - "Ask about selection" mode for highlighted text
   - Mobile responsive, dark mode
   - Session-based history only

## Key Standards

- Security: API keys in env vars, HTTPS only, proper CORS
- Quality: 90%+ accuracy, cite chapters/sections
- Code: Python type hints, TypeScript, ESLint
- Testing: 80%+ coverage, integration tests

## Constraints & Requirements

**Constraints:**
- Gemini: 15 RPM, 1M TPM, 1500 RPD
- Qdrant: 1GB storage, 100 QPS
- Deploy: Vercel/Railway free tier
- Book: Already on GitHub Pages

**Structure requirements:**
- Book embedded in Qdrant
- Backend responds with relevant answers
- Widget on all book pages
- Answers cite book sections
- "Ask about selection" works
- Publicly deployed
- Free tier limits respected

**AI usage rules:**
- Use only free tier APIs (Gemini 1.5 Flash, Qdrant Cloud)
- No user data storage or tracking
- Rate limiting to respect API limits

**Success criteria:**
- ✅ Book embedded in Qdrant
- ✅ Backend responds with relevant answers
- ✅ Widget on all book pages
- ✅ Answers cite book sections
- ✅ "Ask about selection" works
- ✅ Publicly deployed
- ✅ Free tier limits respected

**Out of Scope:**
- User auth, payments, voice I/O, image generation

## Governance
Constitution supersedes all other practices; Amendments require documentation, approval, and a migration plan.
All PRs/reviews must verify compliance; Complexity must be justified.

**Version**: 1.1.0 | **Ratified**: 2025-12-25 | **Last Amended**: 2025-12-25