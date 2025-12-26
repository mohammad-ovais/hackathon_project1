# Integrated RAG Chatbot for Physical AI & Humanoid Robotics Textbook

## 1. Purpose & Scope

### 1.1 Purpose
The purpose of this system is to provide an intelligent, context-aware chatbot that enables learners, educators, and reviewers of the Physical AI & Humanoid Robotics textbook to ask questions and receive accurate answers based on the textbook content. The system uses Retrieval-Augmented Generation (RAG) to ensure responses are grounded in the actual textbook material, preventing hallucinations and providing reliable information with proper citations. This implementation adheres to the project Constitution's core principles of Accuracy, Privacy, Performance, Free Tier Compliance, and Citation.

### 1.2 Scope
**In Scope:**
- Integration of a RAG-based chatbot within the Docusaurus-based textbook website
- Processing of MD/MDX textbook content for vector indexing and retrieval
- Real-time question answering based on textbook content (Accuracy principle)
- "Ask about selection" functionality for highlighted text
- Citation of source chapters/sections in all responses (Citation principle)
- Rate limiting to respect free-tier usage limits (Free Tier Compliance principle)
- Embedding of chatbot widget on all textbook pages
- No user data storage or tracking (Privacy principle)

**Out of Scope:**
- User authentication or account management
- Storage of user data or conversation history (Privacy principle)
- Support for external knowledge sources beyond the textbook
- Voice, image, or multimodal input/output capabilities
- Advanced analytics or user behavior tracking (Privacy principle)
- Content creation beyond the textbook material

### 1.3 Target Audience
- Learners using the Physical AI & Humanoid Robotics textbook
- Educators teaching from the textbook
- Reviewers and researchers in the field
- Anyone seeking to understand concepts from the textbook

## 2. Functional Requirements

### 2.1 Content Processing
- **FR-1.1**: The system shall parse and process MD/MDX files from the Physical AI & Humanoid Robotics textbook
- **FR-1.2**: The system shall chunk textbook content into 500-token segments with 50-token overlap (Constitution requirement)
- **FR-1.3**: The system shall generate vector embeddings for each content chunk using Gemini text-embedding-004 (free tier)
- **FR-1.4**: The system shall index content chunks in Qdrant Cloud vector database with metadata (chapter, section, page URL)

### 2.2 Question Answering
- **FR-2.1**: The system shall accept natural language questions about the textbook content
- **FR-2.2**: The system shall retrieve relevant content chunks from the vector database based on the question
- **FR-2.3**: The system shall generate answers using Gemini 1.5 Flash based only on retrieved content (Accuracy principle - no hallucinations)
- **FR-2.4**: The system shall provide citations to specific chapters/sections where information was found (Citation principle)
- **FR-2.5**: The system shall prevent hallucinations by grounding all responses in retrieved content (Accuracy principle)

### 2.3 Text Selection Feature
- **FR-3.1**: The system shall allow users to highlight text on any textbook page
- **FR-3.2**: The system shall enable users to ask questions specifically about the highlighted text via POST /api/chat/query-selection endpoint
- **FR-3.3**: The system shall prioritize the selected text in its retrieval process
- **FR-3.4**: The system shall provide contextually relevant answers based on the selected text

### 2.4 User Interface
- **FR-4.1**: The system shall embed a ChatKit.js-based widget on all pages of the textbook website
- **FR-4.2**: The system shall provide a clean, intuitive interface for asking questions
- **FR-4.3**: The system shall display answers with clear citations to source material (Citation principle)
- **FR-4.4**: The system shall indicate when no relevant information is found in the textbook
- **FR-4.5**: The system shall support session-based history only, with no persistent storage (Privacy principle)

### 2.5 Rate Limiting and Free Tier Compliance
- **FR-5.1**: The system shall implement rate limiting of ≤10 requests per minute per user (within Constitution limits: 15 RPM)
- **FR-5.2**: The system shall provide appropriate feedback when rate limits are exceeded
- **FR-5.3**: The system shall maintain fair usage across all users
- **FR-5.4**: The system shall operate within Qdrant Cloud free tier limits (≤1GB storage, 100 QPS)
- **FR-5.5**: The system shall use only free tier APIs (Gemini 1.5 Flash)

## 3. Non-Functional Requirements

### 3.1 Performance (Constitution: < 3 second response time)
- **NFR-1.1**: Response latency shall be less than 3 seconds for 95% of requests (Constitution requirement)
- **NFR-1.2**: System shall handle concurrent users without degradation
- **NFR-1.3**: Vector search operations shall complete within 1 second
- **NFR-1.4**: Content indexing operations shall be optimized for query performance

### 3.2 Reliability
- **NFR-2.1**: System shall maintain 99% uptime during operational hours
- **NFR-2.2**: System shall gracefully handle API rate limits and failures
- **NFR-2.3**: Failed requests shall provide informative error messages
- **NFR-2.4**: System shall have appropriate retry mechanisms for transient failures

### 3.3 Scalability and Free Tier Compliance
- **NFR-3.1**: System shall operate within Qdrant Cloud free-tier limits (≤1GB storage, 100 QPS - Constitution requirement)
- **NFR-3.2**: System shall respect Gemini free-tier usage constraints (15 RPM, 1M TPM, 1500 RPD - Constitution requirement)
- **NFR-3.3**: System shall be designed to scale within free-tier limitations
- **NFR-3.4**: Content processing shall be efficient to minimize resource usage

### 3.4 Security and Privacy
- **NFR-4.1**: System shall not store personal user data or conversation history (Privacy principle)
- **NFR-4.2**: API keys and credentials shall be securely managed in environment variables
- **NFR-4.3**: User input shall be validated and sanitized to prevent injection attacks
- **NFR-4.4**: Rate limiting shall prevent abuse and excessive resource consumption
- **NFR-4.5**: HTTPS only connections shall be enforced
- **NFR-4.6**: Proper CORS configuration shall be implemented

### 3.5 Usability
- **NFR-5.1**: Chat interface shall be intuitive and accessible (ChatKit.js patterns)
- **NFR-5.2**: Citations shall be clearly presented and linked to source content (Citation principle)
- **NFR-5.3**: System shall provide helpful feedback when no relevant information is found
- **NFR-5.4**: Error messages shall be user-friendly and informative
- **NFR-5.5**: Mobile responsive design with dark mode support shall be implemented

## 4. System Architecture

### 4.1 Overview
The system follows a microservices architecture with three main components: a data pipeline, a FastAPI backend service, and a React-based frontend widget integrated into the Docusaurus site. This architecture adheres to the Constitution's requirements for data pipeline (Qdrant MCP), backend (FastAPI), and frontend (React Widget) patterns.

### 4.2 Data Pipeline (Constitution requirements)
```
[MD/MDX Files] → [Content Parser] → [Text Chunker (500 tokens, 50-token overlap)] → [Embedding Generator] → [Qdrant Cloud]
```

1. **Content Parser**: Processes Docusaurus MDX files from the textbook repository
2. **Text Chunker**: Segments content into 500-token chunks with 50-token overlap (Constitution requirement)
3. **Embedding Generator**: Creates vector embeddings using Gemini text-embedding-004 (free tier)
4. **Qdrant Cloud**: Stores vectors with metadata (chapter, section, page URL) for efficient retrieval

### 4.3 Backend Architecture (Constitution requirements)
```
[FastAPI Server (async)]
├── [Rate Limiter (≤10 req/min/user)]
├── [Question Parser]
├── [Vector Search Service]
├── [RAG Generator (Gemini 1.5 Flash)]
├── [Citation Service]
└── [Security Layer (API keys, HTTPS, CORS)]
```

1. **Rate Limiter**: Implements user-based rate limiting (≤10 req/min - within Constitution's 15 RPM limit)
2. **Question Parser**: Processes and understands user questions
3. **Vector Search Service**: Queries Qdrant Cloud for relevant content
4. **RAG Generator**: Uses Gemini 1.5 Flash (free tier) to generate answers from retrieved content
5. **Citation Service**: Creates proper citations to source materials (Citation principle)
6. **Security Layer**: Manages API keys in env vars, enforces HTTPS, proper CORS (Security requirements)

### 4.4 Frontend Architecture (Constitution requirements)
```
[Docusaurus Site]
└── [ChatKit.js Widget]
    ├── [Question Input]
    ├── [Response Display with Citations]
    ├── [Text Selection Handler (selected text queries)]
    ├── [Session History (no persistent storage)]
    └── [Citation Formatter]
```

1. **ChatKit.js Widget**: React-based chat interface following ChatKit patterns
2. **Question Input**: Handles user question input
3. **Response Display**: Shows answers with citations (Citation principle)
4. **Text Selection Handler**: Manages "ask about selection" functionality for highlighted text
5. **Session History**: Session-based history only, no persistent storage (Privacy principle)
6. **Citation Formatter**: Formats and links citations to source content

### 4.5 Technology Stack (Constitution compliance)
- **Backend**: FastAPI + async (Constitution requirement)
- **LLM**: Gemini 1.5 Flash (free tier: 15 RPM, 1500 RPD - Constitution requirement)
- **Embeddings**: Gemini text-embedding-004 (free tier)
- **Vector DB**: Qdrant Cloud (free tier: 1GB, 100 QPS - Constitution requirement)
- **Frontend**: ChatKit.js patterns for Docusaurus (Constitution requirement)
- **Documentation**: Docusaurus v3
- **Deployment**: Vercel/Railway (free tier)
- **Features**: Floating button, chat window, typing indicators, mobile responsive, dark mode (Constitution requirement)

### 4.6 API Endpoints (Constitution compliance)
- **POST /api/chat/query**: General questions endpoint
- **POST /api/chat/query-selection**: Selected text questions endpoint
- **GET /api/health**: Health check endpoint

## 5. API Contracts

### 5.1 General Question Answering API
```
POST /api/chat/query
```

**Request Body:**
```json
{
  "question": "string",
  "context_metadata": {
    "page_url": "string",
    "chapter": "string",
    "section": "string"
  }
}
```

**Response:**
```json
{
  "answer": "string",
  "citations": [
    {
      "chapter": "string",
      "section": "string",
      "url": "string",
      "relevance_score": "number"
    }
  ],
  "confidence": "number"
}
```

**Status Codes:**
- 200: Successful response
- 429: Rate limit exceeded (≤10 req/min/user - Constitution requirement)
- 400: Invalid request format
- 500: Internal server error

### 5.2 Selected Text Question Answering API
```
POST /api/chat/query-selection
```

**Request Body:**
```json
{
  "question": "string",
  "selected_text": "string",
  "context_metadata": {
    "page_url": "string",
    "chapter": "string",
    "section": "string"
  }
}
```

**Response:**
```json
{
  "answer": "string",
  "citations": [
    {
      "chapter": "string",
      "section": "string",
      "url": "string",
      "relevance_score": "number"
    }
  ],
  "confidence": "number"
}
```

**Status Codes:**
- 200: Successful response
- 429: Rate limit exceeded (≤10 req/min/user - Constitution requirement)
- 400: Invalid request format
- 500: Internal server error

### 5.3 Health Check API
```
GET /api/health
```

**Response:**
```json
{
  "status": "healthy",
  "timestamp": "ISO 8601 datetime",
  "dependencies": {
    "qdrant": "boolean",
    "gemini": "boolean"
  }
}
```

## 6. Testing Strategy

### 6.1 Unit Testing
- Test individual components of the RAG pipeline
- Validate embedding generation and vector search functionality
- Verify rate limiting mechanisms (≤10 req/min/user)
- Test citation generation and formatting (Citation principle)
- Validate no user data persistence (Privacy principle)
- Test accuracy of responses against textbook content (Accuracy principle)

### 6.2 Integration Testing
- Test end-to-end question answering workflow
- Validate API contract compliance with Constitution endpoints
- Test integration with Qdrant Cloud and Gemini APIs
- Verify text selection functionality (POST /api/chat/query-selection)
- Test adherence to free-tier limits (Qdrant 1GB, 100 QPS; Gemini 15 RPM, 1500 RPD)

### 6.3 Performance Testing
- Measure response latency under various loads (target: <3 seconds - Constitution requirement)
- Test rate limiting effectiveness
- Validate system performance within free-tier constraints
- Test concurrent user handling
- Verify 500-token chunking with 50-token overlap (Constitution requirement)

### 6.4 Functional Testing
- Verify accuracy of answers against textbook content (Accuracy principle)
- Test citation correctness to chapters/sections (Citation principle)
- Validate "ask about selection" functionality
- Test error handling and edge cases
- Verify no hallucinations in responses (Accuracy principle)

### 6.5 Compliance Testing
- Verify no user data storage or tracking (Privacy principle)
- Test free-tier usage limits compliance
- Validate API key security (environment variables)
- Confirm HTTPS and CORS configuration
- Verify session-only history (no persistent storage - Privacy principle)

### 6.6 User Acceptance Testing
- Test with actual textbook users (learners, educators)
- Validate usability and interface design (ChatKit.js patterns)
- Gather feedback on answer quality and relevance
- Test accessibility compliance (WCAG 2.1 AA)
- Validate mobile responsiveness and dark mode

## 7. Acceptance Criteria

### 7.1 Core Functionality (Constitution compliance)
- [ ] System successfully processes and indexes all textbook MD/MDX content using 500-token chunks with 50-token overlap
- [ ] Users can ask questions and receive accurate answers based on textbook content (Accuracy principle)
- [ ] All answers include proper citations to source chapters/sections (Citation principle)
- [ ] "Ask about selection" functionality works for highlighted text via POST /api/chat/query-selection endpoint
- [ ] System prevents hallucinations by grounding responses in retrieved content (Accuracy principle)
- [ ] No user data is stored or tracked (Privacy principle)

### 7.2 Performance (Constitution compliance: < 3 second response time)
- [ ] Response latency is less than 3 seconds for 95% of requests (Constitution requirement)
- [ ] System handles concurrent users without degradation
- [ ] Vector search operations complete within 1 second

### 7.3 Reliability
- [ ] System maintains 99% uptime during operational hours
- [ ] Rate limiting functions correctly (≤10 requests per minute per user, within Constitution's 15 RPM)
- [ ] System gracefully handles API failures and rate limits

### 7.4 Integration (Constitution compliance)
- [ ] ChatKit.js-based widget is embedded on all textbook pages
- [ ] Widget interface follows ChatKit patterns, is intuitive and accessible
- [ ] Citations are clearly presented and linked to source content (Citation principle)
- [ ] Text selection functionality works across all browsers
- [ ] Mobile responsive design with dark mode support is implemented

### 7.5 Free-tier Compliance (Constitution requirements)
- [ ] System operates within Qdrant Cloud free-tier limits (≤1GB storage, 100 QPS)
- [ ] System respects Gemini free-tier usage constraints (15 RPM, 1M TPM, 1500 RPD)
- [ ] Deployment uses free-tier resources only (Vercel/Railway)
- [ ] API keys are securely managed in environment variables

### 7.6 Privacy and Security (Constitution compliance)
- [ ] No personal user data is stored or tracked (Privacy principle)
- [ ] Session-based history only, with no persistent storage (Privacy principle)
- [ ] HTTPS connections are enforced
- [ ] Proper CORS configuration is implemented
- [ ] User input is validated and sanitized to prevent injection attacks

## 8. Governance & Compliance

### 8.1 Data Governance (Constitution: Privacy principle)
- No personal user data is stored or processed (Privacy principle)
- Conversation history is not retained (Privacy principle)
- Only textbook content is used as knowledge source
- All processing is done in real-time without data persistence (Privacy principle)
- Session-based history only, with no persistent storage (Privacy principle)

### 8.2 API Management (Constitution: Free Tier Compliance)
- API keys are securely managed in environment variables (Security requirement)
- Rate limiting prevents abuse and excessive usage (≤10 req/min/user)
- API usage is monitored to ensure compliance with free-tier limits (Qdrant 1GB, 100 QPS; Gemini 15 RPM, 1500 RPD)
- Proper error handling prevents credential exposure
- HTTPS connections are enforced with proper CORS configuration

### 8.3 Content Integrity (Constitution: Accuracy principle)
- All responses are grounded in actual textbook content (Accuracy principle)
- Citations ensure transparency and verifiability (Citation principle)
- System prevents hallucinations through content validation (Accuracy principle)
- Content accuracy is maintained through source verification (Accuracy principle)
- 500-token chunking with 50-token overlap ensures context preservation

### 8.4 Citation Standards (Constitution: Citation principle)
- All responses include citations to specific chapters/sections (Citation principle)
- Citations are linked to source content for verification (Citation principle)
- Source metadata (chapter, section, page URL) is maintained in Qdrant Cloud
- Citation accuracy is validated during response generation

### 8.5 Accessibility (Constitution compliance)
- Chat interface follows WCAG 2.1 AA guidelines
- Screen reader compatibility is maintained
- Keyboard navigation is supported
- Color contrast meets accessibility standards
- Mobile responsive design with dark mode support is implemented

### 8.6 Open Source Compliance
- All dependencies use appropriate open source licenses
- Attribution requirements are met
- No proprietary restrictions on textbook content usage
- System is designed for public accessibility

### 8.7 Constitution Adherence
- This specification adheres to all core principles: Accuracy, Privacy, Performance, Free Tier Compliance, and Citation
- All implementation must verify compliance with Constitution requirements
- Any deviations require documented amendments to the Constitution
- The Constitution supersedes all other practices and requirements

## 9. Implementation Plan

### 9.1 Phase 1: Infrastructure Setup (Constitution compliance)
- Set up Qdrant Cloud instance (≤1GB storage limit)
- Configure Gemini API access (free tier: 15 RPM, 1500 RPD)
- Deploy FastAPI backend with async support
- Implement security layer (API keys in env vars, HTTPS, CORS)
- Integrate with Docusaurus site

### 9.2 Phase 2: Core RAG Pipeline (Constitution compliance)
- Implement content parsing from Docusaurus MDX files
- Develop 500-token chunking with 50-token overlap (Constitution requirement)
- Create embedding generation using Gemini text-embedding-004 (free tier)
- Build vector indexing with chapter/section metadata
- Create vector search functionality (≤100 QPS for free tier)
- Build RAG answer generation with hallucination prevention (Accuracy principle)
- Implement citation service (Citation principle)

### 9.3 Phase 3: User Interface (Constitution compliance)
- Integrate ChatKit.js widget on all textbook pages
- Implement "ask about selection" functionality for highlighted text
- Add citation display with chapter/section links (Citation principle)
- Implement rate limiting interface (≤10 req/min/user)
- Add mobile responsive design and dark mode support
- Implement session-based history only (Privacy principle)

### 9.4 Phase 4: Testing & Deployment (Constitution compliance)
- Conduct comprehensive testing (unit, integration, performance, compliance)
- Verify <3 second response time (Performance requirement)
- Perform user acceptance testing with textbook users
- Test free-tier usage limits compliance
- Deploy to production on Vercel/Railway (free tier)
- Monitor and optimize performance within free-tier constraints

## 10. Success Metrics

- User satisfaction with answer accuracy (Accuracy principle)
- Response time performance metrics (<3 seconds - Performance requirement)
- API usage within free-tier limits (Qdrant 1GB, 100 QPS; Gemini 15 RPM, 1500 RPD)
- User engagement and adoption rates
- Citation accuracy and helpfulness (Citation principle)
- Privacy compliance (no user data storage - Privacy principle)
- Constitution adherence verification