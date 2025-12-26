---
id: 1
title: Create RAG Chatbot Constitution
stage: constitution
date: 2025-12-25
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: main
user: Claude
command: /sp.constitution
labels: [constitution,rag-chatbot,physical-ai,robotics]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 - none
---

## Prompt

```text
create --project "rag-chatbot" Project: RAG Chatbot for Physical AI & Humanoid Robotics Textbook

Core Principles:
- Accuracy: Answers only from book content, no hallucinations
- Privacy: No user data storage, minimal tracking
- Performance: < 3 second response time
- Free tier only: No paid APIs

Architecture:

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

Key Standards:
- Security: API keys in env vars, HTTPS only, proper CORS
- Quality: 90%+ accuracy, cite chapters/sections
- Code: Python type hints, TypeScript, ESLint
- Testing: 80%+ coverage, integration tests

Constraints:
- Gemini: 15 RPM, 1M TPM, 1500 RPD
- Qdrant: 1GB storage, 100 QPS
- Deploy: Vercel/Railway free tier
- Book: Already on GitHub Pages

Success Criteria (MVP):
✅ Book embedded in Qdrant
✅ Backend responds with relevant answers
✅ Widget on all book pages
✅ Answers cite book sections
✅ "Ask about selection" works
✅ Publicly deployed
✅ Free tier limits respected

Out of Scope:
❌ User auth, payments, voice I/O, image generation
```

## Response snapshot

Updated the project constitution file (.specify/memory/constitution.md) with specific details for the RAG Chatbot project for Physical AI & Humanoid Robotics Textbook. The new constitution includes:

- Core principles: Accuracy, Privacy, Performance, Free Tier Compliance, and Citation
- Architecture sections for Data Pipeline, Backend, and Frontend
- Key standards for security, quality, code, and testing
- Constraints and requirements specific to the RAG chatbot project
- Success criteria and out-of-scope items

The constitution now properly reflects the RAG chatbot project instead of the previous book creation project.

## Outcome

- ✅ Impact: Project constitution updated to reflect RAG chatbot project requirements
- 🧪 Tests: none
- 📁 Files: .specify/memory/constitution.md updated
- 🔁 Next prompts: Plan and implement RAG chatbot features
- 🧠 Reflection: Constitution now properly guides the RAG chatbot development with appropriate principles and constraints

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin feature specification for RAG chatbot