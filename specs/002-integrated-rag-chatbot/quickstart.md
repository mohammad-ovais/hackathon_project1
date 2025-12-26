# Quickstart Guide: Integrated RAG Chatbot

## Prerequisites
- Python 3.11+
- Node.js 18+
- Docusaurus project for the textbook
- Qdrant Cloud account (free tier)
- Google AI API key (Gemini free tier)

## 1. Environment Setup

### 1.1 Clone and Navigate
```bash
git clone <repository-url>
cd <repository-name>
cd specs/002-integrated-rag-chatbot
```

### 1.2 Backend Setup
```bash
cd backend
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install -r requirements.txt
```

### 1.3 Frontend Setup
```bash
cd frontend
npm install
```

### 1.4 Environment Variables
Create `.env` file in backend directory:
```env
GEMINI_API_KEY=your_gemini_api_key
QDRANT_URL=your_qdrant_cloud_url
QDRANT_API_KEY=your_qdrant_api_key
```

## 2. Data Ingestion

### 2.1 Process Textbook Content
```bash
cd ingestion
python -m venv venv
source venv/bin/activate
pip install -r requirements.txt

# Run the ingestion pipeline
python src/ingestion_pipeline.py --source-path ../../docusaurus/docs --chunk-size 500 --overlap 50
```

### 2.2 Verify Vector Storage
Check that textbook content has been embedded in Qdrant Cloud.

## 3. Backend Service

### 3.1 Start Backend Server
```bash
cd backend
source venv/bin/activate
uvicorn main:app --reload --port 8000
```

### 3.2 Test API Endpoints
```bash
# Health check
curl http://localhost:8000/api/health

# Test query
curl -X POST http://localhost:8000/api/chat/query \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is Physical AI?",
    "session_id": "test-session-123",
    "include_citations": true
  }'
```

## 4. Frontend Integration

### 4.1 Build Widget
```bash
cd frontend
npm run build
```

### 4.2 Integrate with Docusaurus
Add the chat widget to your Docusaurus site by including the JavaScript file in your `docusaurus.config.js`:

```js
module.exports = {
  // ... other config
  scripts: [
    {
      src: '/js/chat-widget.js',
      async: true,
    },
  ],
};
```

### 4.3 Initialize Widget
```javascript
// Initialize the widget after page load
window.ChatWidget.init({
  backendUrl: 'http://localhost:8000',
  theme: 'light',
  position: 'right'
});
```

## 5. Testing the Complete Flow

### 5.1 End-to-End Test
1. Start backend: `uvicorn main:app --reload --port 8000`
2. Build frontend widget: `npm run build`
3. Start Docusaurus: `npm run start`
4. Navigate to a textbook page
5. Use the chat widget to ask questions about the content
6. Verify citations appear correctly

### 5.2 API Testing
```bash
# Test general query
curl -X POST http://localhost:8000/api/chat/query \
  -H "Content-Type: application/json" \
  -d '{
    "query": "Explain the key concepts of humanoid robotics",
    "session_id": "test-session-456",
    "include_citations": true
  }'

# Test selected text query
curl -X POST http://localhost:8000/api/chat/query-selection \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What does this mean?",
    "selected_text": "Physical AI combines principles of physics with artificial intelligence",
    "session_id": "test-session-456",
    "include_citations": true
  }'
```

## 6. Production Deployment

### 6.1 Backend Deployment
Deploy the FastAPI backend to a service that supports Python (Railway, Vercel, etc.)

### 6.2 Frontend Deployment
The widget will be served with the Docusaurus site on GitHub Pages.

### 6.3 Environment Configuration
Set environment variables for production:
- `GEMINI_API_KEY`: Production Gemini API key
- `QDRANT_URL`: Production Qdrant Cloud URL
- `QDRANT_API_KEY`: Production Qdrant API key