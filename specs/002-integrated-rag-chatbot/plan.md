# Implementation Plan: Integrated RAG Chatbot for Physical AI & Humanoid Robotics Textbook

**Branch**: `002-integrated-rag-chatbot` | **Date**: 2025-12-26 | **Spec**: [specs/002-integrated-rag-chatbot/spec.md](/specs/002-integrated-rag-chatbot/spec.md)
**Input**: Feature specification from `/specs/002-integrated-rag-chatbot/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implementation of an integrated RAG (Retrieval-Augmented Generation) chatbot that allows users to ask questions about the Physical AI & Humanoid Robotics textbook content. The solution uses a FastAPI backend with Qdrant Cloud vector database and Gemini API for processing textbook content, with a frontend widget embedded in Docusaurus book pages. The system operates within free tier limits while providing accurate, cited responses with <3 second latency.

## Technical Context

**Language/Version**: Python 3.11 (backend), JavaScript/TypeScript (frontend)
**Primary Dependencies**: FastAPI, Gemini API, Qdrant Cloud, Docusaurus, React
**Storage**: Qdrant Cloud (vector database), in-memory session storage
**Testing**: pytest (backend), Jest/React Testing Library (frontend)
**Target Platform**: Linux server (backend), Web browsers (frontend)
**Project Type**: Web (frontend + backend)
**Performance Goals**: <3 second response time, 90%+ accuracy on textbook questions
**Constraints**: Free tier limits (Gemini: 15 RPM, 1500 RPD; Qdrant: 1GB, 100 QPS), no persistent user data storage
**Scale/Scope**: Single textbook content, multiple concurrent users, 1GB vector storage limit

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Accuracy Compliance
✅ Answers will be grounded only in book content using RAG approach
✅ No hallucinations as responses will be based on retrieved textbook passages

### Privacy Compliance
✅ No user data will be stored persistently
✅ Minimal tracking with in-memory session only
✅ Query logs will not be retained

### Performance Compliance
✅ Target <3 second response time aligns with requirement
✅ Will implement performance monitoring to ensure compliance

### Free Tier Compliance
✅ Using Gemini 1.5 Flash free tier (15 RPM, 1500 RPD)
✅ Using Qdrant Cloud free tier (1GB storage)
✅ Implementing rate limiting (10 req/min/user) to respect limits

### Citation Compliance
✅ Answers will include citations to specific chapters/sections
✅ Metadata will include chapter, section, and page URL information

## Project Structure

### Documentation (this feature)

```text
specs/002-integrated-rag-chatbot/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── src/
│   ├── models/
│   ├── services/
│   ├── api/
│   └── utils/
├── requirements.txt
├── main.py
└── tests/

frontend/
├── src/
│   ├── components/
│   ├── services/
│   └── utils/
├── chat-widget.js
└── tests/

docusaurus/
├── src/
│   ├── components/
│   └── pages/
└── static/

ingestion/
├── src/
│   ├── chunker.py
│   ├── embeddings.py
│   └── qdrant_client.py
└── requirements.txt
```

**Structure Decision**: Web application with separate backend (FastAPI) and frontend (React widget) components. The backend handles RAG logic and API endpoints, while the frontend provides the chat widget for Docusaurus book pages. An ingestion pipeline handles document processing and vector storage.

## Architecture Sketch

### High-Level System Architecture
```
┌─────────────────────────────────────────────────────────────────┐
│                    Docusaurus Book (GitHub Pages)               │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐ │
│  │   Chapter 1     │  │   Chapter 2     │  │   Chapter N     │ │
│  │                 │  │                 │  │                 │ │
│  │ [Chat Widget]   │  │ [Chat Widget]   │  │ [Chat Widget]   │ │
│  └─────────────────┘  └─────────────────┘  └─────────────────┘ │
└─────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────┐
│                   Frontend Chatbot Widget (ChatKit.js)          │
│  ┌─────────────────────────────────────────────────────────────┐ │
│  │  Floating Button → Chat Window → API Requests             │ │
│  │  - General Questions                                       │ │
│  │  - Selected Text Questions                                 │ │
│  │  - Citations Display                                       │ │
│  └─────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────┐
│                    FastAPI Backend (Agent-style)                │
│  ┌─────────────────────────────────────────────────────────────┐ │
│  │  API Endpoints:                                           │ │
│  │  - POST /api/chat/query (general questions)               │ │
│  │  - POST /api/chat/query-selection (selected text)         │ │
│  │  - GET /api/health                                        │ │
│  │  Services:                                                │ │
│  │  - RAG Orchestration                                      │ │
│  │  - Rate Limiting                                          │ │
│  │  - Session Management                                     │ │
│  └─────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────┘
                                    │
                    ┌───────────────┼───────────────┐
                    ▼               ▼               ▼
    ┌────────────────────────┐ ┌─────────────────┐ ┌─────────────────┐
    │    Gemini API          │ │  Qdrant Cloud   │ │  Rate Limiting  │
    │  (1.5 Flash, Free)     │ │  (Vector DB)    │ │  (In-memory)    │
    │  - Embeddings          │ │  - Text chunks  │ │  - Session IDs  │
    │  - Generation          │ │  - Metadata     │ │  - Request log  │
    └────────────────────────┘ └─────────────────┘ └─────────────────┘
```

### Data Flow
1. **Book ingestion → embeddings → Qdrant**:
   - Textbook content from Docusaurus MDX files
   - Chunked into 500-token segments with 50-token overlap
   - Processed through Gemini text-embedding-004 model
   - Stored in Qdrant Cloud with metadata (chapter, section, URL)

2. **User query / selected text → retrieval → generation → response**:
   - User asks question via chat widget
   - Query sent to backend API with session ID
   - Query embedded using Gemini model
   - Top-3 relevant chunks retrieved from Qdrant
   - Context and query sent to Gemini for response generation
   - Citations generated from retrieved chunk metadata
   - Response returned to frontend widget

### Free-tier Boundaries and Rate-limiting Points
- **Gemini API**: 15 requests per minute (RPM), 1500 requests per day (RPD)
- **Qdrant Cloud**: 1GB storage, 100 queries per second (QPS)
- **Rate limiting**: 10 requests per minute per session ID
- **Response timeout**: 30 seconds to prevent hanging requests

## Section & Component Structure

### Data Ingestion Pipeline Components
- `ingestion/src/chunker.py`: Text chunking logic (500 tokens + 50 overlap)
- `ingestion/src/embeddings.py`: Gemini embedding integration
- `ingestion/src/qdrant_client.py`: Vector database operations
- `ingestion/requirements.txt`: Python dependencies

### Backend Services and API Endpoints
- `backend/src/models/`: Pydantic models for requests/responses
- `backend/src/services/`: RAG orchestration, retrieval, generation
- `backend/src/api/`: FastAPI routes and endpoints
- `backend/src/utils/`: Rate limiting, session management
- `backend/main.py`: Application entry point
- `backend/requirements.txt`: Python dependencies

### Frontend Widget Modules
- `frontend/src/components/`: React components for chat interface
- `frontend/src/services/`: API communication, session management
- `frontend/src/utils/`: Text selection, UI utilities
- `frontend/chat-widget.js`: Embeddable JavaScript widget
- `frontend/package.json`: JavaScript dependencies

### Configuration and Environment Management
- Environment variables for API keys and service URLs
- Configuration files for chunk size, top-K retrieval, rate limits
- Docker configurations for deployment (if needed)

### Folder and Responsibility Boundaries
- `backend/`: All server-side logic, API endpoints, business logic
- `frontend/`: All client-side code, UI components, widget integration
- `ingestion/`: Data processing pipeline, vector database population
- `docusaurus/`: Existing textbook content and Docusaurus site

## Research Approach

### Research-Concurrent Implementation
- Validate Gemini embedding quality and RAG accuracy during implementation
- Test Qdrant Cloud performance and connectivity throughout development
- Iterate on chunk size and retrieval strategies based on results
- Monitor free-tier usage and adjust parameters as needed

### Validation Focus Areas
- Embedding quality: Test semantic similarity of retrieved chunks
- RAG accuracy: Validate response quality against textbook content
- Qdrant behavior: Monitor query performance and storage limits
- Rate limiting: Ensure compliance with free-tier constraints

### Documentation Strategy
- Reference Gemini API documentation for best practices
- Consult Qdrant Cloud documentation for optimal configuration
- Follow FastAPI best practices for API design
- Use React/JavaScript patterns for widget implementation

## Implementation Phases

### Phase 1: Research
- Validate Gemini free-tier limits with sample queries
- Test Qdrant Cloud connectivity and indexing performance
- Prototype embedding and retrieval quality with sample textbook content
- Establish baseline metrics for accuracy and response time

### Phase 2: Foundation
- Implement ingestion pipeline for processing textbook content
- Initialize FastAPI backend with basic API endpoints
- Define API contracts and data models
- Create basic frontend widget shell with minimal functionality
- Set up rate limiting and session management

### Phase 3: Analysis
- Integrate full RAG flow (retrieve + generate)
- Add citation generation from chunk metadata
- Implement selected-text question handling
- Enforce rate limits and API constraints
- Add error handling and fallback strategies

### Phase 4: Synthesis
- Embed widget across all book pages in Docusaurus
- Optimize latency through caching and connection pooling
- Prepare deployment configuration for free tier
- Final validation against acceptance criteria
- Documentation and deployment guide creation

## Decisions Needing Documentation

### Chunk Size and Overlap Choices
- **Options considered**: 250/50, 500/50 (selected), 1000/100 tokens
- **Chosen option**: 500 tokens with 50-token overlap
- **Tradeoffs and rationale**: Balance between context retention and retrieval precision; follows constitution guidelines

### Top-K Retrieval Strategy
- **Options considered**: Top-1, Top-3 (selected), Top-5
- **Chosen option**: Top-3 retrieval
- **Tradeoffs and rationale**: Provides sufficient context without overwhelming the model; optimal for response quality

### Citation Strategy
- **Options considered**: Chapter-only, Chapter+Section (selected), Full path
- **Chosen option**: Include chapter, section, and page URL
- **Tradeoffs and rationale**: Meets constitution requirement for proper citation while remaining concise

### Rate-limiting Approach
- **Options considered**: IP-based, Session-based (selected), User-based
- **Chosen option**: Session-based with 10 req/min/user
- **Tradeoffs and rationale**: Respects free-tier limits while providing fair access; no user identification required

### Free-tier Tradeoffs (Accuracy vs Latency)
- **Options considered**: Higher quality/longer latency, Balanced (selected), Lower quality/faster
- **Chosen option**: Balanced approach using Gemini 1.5 Flash
- **Tradeoffs and rationale**: Optimal balance between cost, quality, and response time for free tier

## Quality Validation & Testing Strategy

### Validation Checks Aligned with Acceptance Criteria
- **Accuracy validation**: Compare responses against textbook content for grounding
- **Citation correctness**: Verify citations point to actual textbook sections
- **Selected-text query correctness**: Test that selected text queries work properly
- **Performance checks**: Ensure responses delivered in <3 seconds
- **Free-tier limit compliance**: Monitor API usage against limits
- **Failure handling**: Test rate limits, empty retrievals, API errors

### Unit Test Focus Areas
- Embedding generation and similarity calculations
- Chunk retrieval algorithms
- Response generation with citations
- Rate limiting logic
- Session management

### Integration Test Checkpoints
- End-to-end RAG pipeline functionality
- API endpoint behavior with various inputs
- Frontend-backend communication
- Widget integration with Docusaurus pages
- Error handling across service boundaries

### Manual Validation Steps
- Test various question types against textbook content
- Verify citation accuracy and completeness
- Check selected text functionality
- Validate rate limiting behavior
- Confirm deployment works correctly

## Governance & Compliance

### Alignment with Constitution Principles
- **Accuracy**: Responses grounded in textbook content using RAG approach
- **Privacy**: No persistent user data storage, in-memory sessions only
- **Performance**: Target <3 second response time with monitoring
- **Free-tier compliance**: All services operate within free tier limits
- **Citation**: All responses include chapter/section citations

### Deviation Documentation
- Any deviations from the constitution will be documented in Architecture Decision Records (ADRs)
- Performance adjustments will be recorded if <3s target cannot be met
- Changes to citation format will be documented if needed for technical reasons
