# Research Plan: Integrated RAG Chatbot

## Phase 0: Research & Validation

### 1. Gemini Free-tier Limits Validation
**Decision**: Use Gemini 1.5 Flash for RAG implementation
**Rationale**: Free tier with sufficient limits for hackathon project (15 RPM, 1500 RPD, 1M TPM)
**Alternatives considered**:
- OpenAI GPT models (would require paid API key)
- Open-source models (would require more infrastructure)
- Gemini Pro (higher cost than free tier)

### 2. Qdrant Cloud Integration
**Decision**: Use Qdrant Cloud free tier for vector storage
**Rationale**: 1GB storage limit sufficient for textbook content, managed service, good Python SDK
**Alternatives considered**:
- Pinecone (would require paid tier for similar limits)
- Weaviate (self-hosting complexity)
- Local vector storage (not suitable for deployed solution)

### 3. Embedding Quality Testing
**Decision**: Use Gemini text-embedding-004 model for embeddings
**Rationale**: Compatible with Gemini generation models, free tier available
**Alternatives considered**:
- OpenAI embeddings (would require paid API)
- Sentence Transformers (local processing, but accuracy unknown)

### 4. Chunk Size and Overlap Strategy
**Decision**: Use 500-token chunks with 50-token overlap based on constitution
**Rationale**: Balance between context retention and retrieval efficiency
**Alternatives considered**:
- Smaller chunks (might lose context)
- Larger chunks (might exceed token limits)

### 5. Top-K Retrieval Strategy
**Decision**: Use Top-3 retrieval for initial implementation
**Rationale**: Balance between relevance and response quality
**Alternatives considered**:
- Top-1 (too restrictive)
- Top-5 (might include less relevant results)

### 6. Frontend Widget Integration
**Decision**: Create ChatKit.js-style widget for Docusaurus integration
**Rationale**: Similar to existing patterns, lightweight, embeddable
**Alternatives considered**:
- Full iframe embedding (more complex)
- Native Docusaurus plugin (more development time)

### 7. Rate Limiting Implementation
**Decision**: Implement client-side and server-side rate limiting
**Rationale**: Prevent API limit breaches, ensure fair usage
**Alternatives considered**:
- No rate limiting (would violate free tier terms)
- Only server-side (less responsive for users)

### 8. Citation Strategy
**Decision**: Include chapter/section references and page URLs in responses
**Rationale**: Meets constitution requirement for citation compliance
**Alternatives considered**:
- No citations (violates constitution)
- Only general references (insufficient accuracy)