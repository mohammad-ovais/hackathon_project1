# Data Model: Integrated RAG Chatbot

## 1. Document Chunk Entity
**Description**: Represents a chunk of textbook content with embeddings

```python
class DocumentChunk:
    id: str                    # Unique identifier for the chunk
    content: str              # Text content of the chunk (max 500 tokens)
    embedding: List[float]    # Vector embedding of the content
    metadata: ChunkMetadata   # Additional metadata for citations
    created_at: datetime      # Timestamp of creation
```

**Fields**:
- `id`: UUID for unique identification
- `content`: The actual text content (500-token chunks with 50-token overlap)
- `embedding`: 768/1536-dimensional vector from Gemini embedding model
- `metadata`: Contains chapter, section, page URL, and source file info
- `created_at`: Timestamp for tracking

## 2. Chunk Metadata Entity
**Description**: Metadata for citations and source tracking

```python
class ChunkMetadata:
    chapter: str              # Chapter name/number
    section: str              # Section name/number
    page_url: str             # URL to the page containing this content
    source_file: str          # Original source file path
    token_count: int          # Number of tokens in the chunk
    position: int             # Position within the document
```

## 3. Query Request Entity
**Description**: Structure for incoming query requests

```python
class QueryRequest:
    query: str                # User's question
    selected_text: Optional[str]  # Text selected by user (if applicable)
    session_id: str           # Session identifier for rate limiting
    include_citations: bool   # Whether to include citations (default: True)
```

## 4. Query Response Entity
**Description**: Structure for response to query requests

```python
class QueryResponse:
    answer: str               # Generated answer to the query
    citations: List[Citation] # List of citations for the answer
    sources: List[SourceChunk] # Source chunks used for generation
    response_time: float      # Time taken to generate response (ms)
```

## 5. Citation Entity
**Description**: Reference to specific parts of the textbook

```python
class Citation:
    chapter: str              # Chapter name/number
    section: str              # Section name/number
    page_url: str             # URL to the referenced page
    text_snippet: str         # Short snippet of the cited text
    confidence: float         # Confidence score (0.0-1.0)
```

## 6. Source Chunk Entity
**Description**: Details about chunks used in response generation

```python
class SourceChunk:
    content: str              # Content of the source chunk
    metadata: ChunkMetadata   # Metadata for the source chunk
    relevance_score: float    # How relevant this chunk was to the query (0.0-1.0)
```

## 7. Rate Limit Entity
**Description**: Track API usage for rate limiting

```python
class RateLimitRecord:
    session_id: str           # User session identifier
    request_count: int        # Number of requests in the current window
    window_start: datetime    # Start time of the rate limit window
    last_request: datetime    # Time of last request
```

## 8. Session Entity
**Description**: Temporary session data (not persisted)

```python
class Session:
    session_id: str           # Unique session identifier
    history: List[ChatMessage] # Chat history for context (in-memory only)
    created_at: datetime      # Session creation time
    last_activity: datetime   # Last interaction time
```

## 9. Chat Message Entity
**Description**: Individual chat message in session history

```python
class ChatMessage:
    role: str                 # 'user' or 'assistant'
    content: str              # Message content
    timestamp: datetime       # When the message was created
```

## Validation Rules
1. **DocumentChunk**: Content must be between 10 and 500 tokens
2. **QueryRequest**: Query must not be empty, session_id must be valid
3. **RateLimitRecord**: Enforce 10 requests per minute per session
4. **Citation**: Must reference valid chapter/section in the textbook

## Relationships
- `QueryResponse` contains multiple `Citation` and `SourceChunk` objects
- `DocumentChunk` has `ChunkMetadata`
- `Session` contains multiple `ChatMessage` objects
- `RateLimitRecord` is associated with a `session_id`