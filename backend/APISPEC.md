# RAG Chatbot API Documentation

## Base URL
All API endpoints are prefixed with `/api`.

## Authentication
Authentication is handled via session IDs. No API key is required for basic functionality.

## Rate Limiting
All endpoints implement rate limiting (10 requests per minute per session).

## Endpoints

### Health Check
```
GET /health
```

#### Description
Check the health status of the API service.

#### Response
```json
{
  "status": "healthy",
  "timestamp": "2025-12-26T00:00:00Z",
  "services": {
    "qdrant": "healthy",
    "gemini": "healthy"
  }
}
```

### Chat Query
```
POST /chat/query
```

#### Description
Submit a question about the textbook content.

#### Request Body
```json
{
  "query": "string, required - User's question about the textbook",
  "session_id": "string, required - Session identifier for rate limiting",
  "include_citations": "boolean, optional - Whether to include citations (default: true)"
}
```

#### Response (200 OK)
```json
{
  "answer": "string - Generated answer to the query",
  "citations": [
    {
      "chapter": "string - Chapter name/number",
      "section": "string - Section name/number",
      "page_url": "string - URL to the referenced page",
      "text_snippet": "string - Short snippet of the cited text",
      "confidence": "number - Confidence score (0.0-1.0)"
    }
  ],
  "sources": [
    {
      "content": "string - Content of the source chunk",
      "metadata": {
        "chapter": "string",
        "section": "string",
        "page_url": "string",
        "source_file": "string",
        "token_count": "integer",
        "position": "integer"
      },
      "relevance_score": "number - How relevant this chunk was to the query (0.0-1.0)"
    }
  ],
  "response_time": "number - Time taken to generate response (ms)"
}
```

#### Error Responses
- `400 Bad Request`: Invalid request format
- `429 Too Many Requests`: Rate limit exceeded
- `500 Internal Server Error`: Server error

### Chat Query with Selected Text
```
POST /chat/query-selection
```

#### Description
Submit a question specifically about selected text.

#### Request Body
```json
{
  "query": "string, required - User's question about the selected text",
  "selected_text": "string, required - Text that was selected by the user",
  "session_id": "string, required - Session identifier for rate limiting",
  "include_citations": "boolean, optional - Whether to include citations (default: true)"
}
```

#### Response (200 OK)
Same as Chat Query endpoint.

#### Error Responses
- `400 Bad Request`: Invalid request format (missing selected_text)
- `429 Too Many Requests`: Rate limit exceeded
- `500 Internal Server Error`: Server error

## Rate Limiting Headers
All responses include the following headers:

- `X-RateLimit-Limit`: Request limit per window
- `X-RateLimit-Remaining`: Remaining requests in current window
- `X-RateLimit-Reset`: Unix timestamp for window reset

## Error Response Format
All error responses follow this format:

```json
{
  "error": {
    "code": "string - Error code",
    "message": "string - Human-readable error message",
    "details": "object - Additional error details (optional)"
  }
}
```

### Common Error Codes
- `INVALID_REQUEST`: Request validation failed
- `RATE_LIMIT_EXCEEDED`: Rate limit exceeded
- `VECTOR_DB_ERROR`: Error connecting to or querying vector database
- `AI_SERVICE_ERROR`: Error with AI service (Gemini)
- `INTERNAL_ERROR`: Internal server error

## Example Usage

### Submitting a Query
```bash
curl -X POST http://localhost:8000/api/chat/query \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is Physical AI?",
    "session_id": "user-session-123",
    "include_citations": true
  }'
```

### Submitting a Query with Selected Text
```bash
curl -X POST http://localhost:8000/api/chat/query-selection \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What does this mean?",
    "selected_text": "Physical AI combines principles of physics with artificial intelligence",
    "session_id": "user-session-123",
    "include_citations": true
  }'
```

## Performance
- Response times typically under 3 seconds
- Supports concurrent users
- Implements graceful degradation when services are slow