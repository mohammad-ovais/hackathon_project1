# Integrated RAG Chatbot for Physical AI & Humanoid Robotics Textbook

This project implements an integrated RAG (Retrieval-Augmented Generation) chatbot that allows users to ask questions about the Physical AI & Humanoid Robotics textbook content. The chatbot provides accurate answers with citations to specific parts of the textbook.

## Table of Contents
- [Features](#features)
- [Architecture](#architecture)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Configuration](#configuration)
- [Running the Application](#running-the-application)
- [API Documentation](#api-documentation)
- [Testing](#testing)
- [Deployment](#deployment)
- [Troubleshooting](#troubleshooting)

## Features

- **Natural Language Queries**: Ask questions about the textbook content in plain English
- **Accurate Responses**: Answers grounded in textbook content using RAG approach
- **Citation Generation**: All answers include citations to specific chapters/sections
- **Selected Text Queries**: Ask questions about specific text passages you've highlighted
- **Real-time Responses**: Answers typically returned in under 3 seconds
- **Privacy Focused**: No user data is persisted
- **Embeddable Widget**: JavaScript widget that integrates seamlessly with Docusaurus sites
- **Rate Limited**: Respects free-tier API limits

## Architecture

The application consists of three main components:

1. **Backend**: FastAPI service handling RAG logic and API endpoints
2. **Frontend**: React widget for Docusaurus book pages
3. **Ingestion**: Pipeline for processing textbook content into vector database

### Tech Stack
- **Backend**: Python 3.11, FastAPI, Google Generative AI (Gemini), Qdrant
- **Frontend**: JavaScript, React, Webpack
- **Vector Database**: Qdrant Cloud (free tier)
- **Documentation**: Docusaurus
- **Containerization**: Docker, Docker Compose

## Prerequisites

- Python 3.11+
- Node.js 18+
- Docker and Docker Compose (for deployment)
- Google AI API key (Gemini free tier)
- Qdrant Cloud account (free tier)

## Installation

### 1. Clone the Repository

```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Backend Setup

```bash
# Navigate to backend directory
cd backend

# Create virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt
```

### 3. Frontend Setup

```bash
# Navigate to frontend directory
cd frontend

# Install dependencies
npm install
```

### 4. Ingestion Pipeline Setup

```bash
# Navigate to ingestion directory
cd ingestion

# Create virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt
```

## Configuration

### 1. Environment Variables

Create a `.env` file in the backend directory with the following variables:

```env
# Gemini API Configuration
GEMINI_API_KEY=your_gemini_api_key_here

# Qdrant Configuration
QDRANT_URL=your_qdrant_cloud_url_here
QDRANT_API_KEY=your_qdrant_api_key_here
QDRANT_COLLECTION_NAME=textbook_chunks

# Application Configuration
QDRANT_COLLECTION_NAME=textbook_chunks
```

### 2. Docusaurus Integration

The chatbot widget is automatically integrated with Docusaurus via the plugin. Ensure your `docusaurus.config.js` includes the plugin:

```js
plugins: [
  [
    './src/plugins/chatbot-plugin.js',
    {
      backendUrl: 'http://localhost:8000', // Update to your backend URL
      theme: 'light',
      position: 'right'
    }
  ]
],
```

## Running the Application

### Development Mode

#### 1. Start the Backend Service

```bash
cd backend
source venv/bin/activate  # Activate virtual environment
uvicorn main:app --reload --port 8000
```

#### 2. Process Textbook Content (One-time setup)

```bash
cd ingestion
source venv/bin/activate  # Activate virtual environment

# Run the ingestion pipeline
python src/ingestion_pipeline.py --source-path ../docusaurus/docs --chunk-size 500 --overlap 50
```

#### 3. Start Docusaurus

```bash
cd docusaurus
npm start
```

### Production Mode with Docker

```bash
# Build and start all services
docker-compose up --build
```

## API Documentation

The backend API provides the following endpoints:

### Health Check
- `GET /api/health` - Check service health

### Chat Endpoints
- `POST /api/chat/query` - General questions about textbook
- `POST /api/chat/query-selection` - Questions about selected text

For detailed API documentation, see [APISPEC.md](./backend/APISPEC.md).

## Testing

### Backend Tests

```bash
cd backend
source venv/bin/activate
python -m pytest tests/
```

### Load Testing

```bash
cd backend
python test_load.py
```

### Rate Limit Testing

```bash
cd backend
python test_rate_limit.py
```

## Deployment

### Production Deployment

1. Update the `.env` file with production credentials
2. Build the frontend widget:
   ```bash
   cd frontend
   npm run build
   ```
3. Deploy using Docker Compose:
   ```bash
   docker-compose up -d --build
   ```

### Docusaurus Integration

The widget is automatically integrated with Docusaurus. Just build and deploy your Docusaurus site as usual.

## Troubleshooting

### Common Issues

1. **API Keys**: Ensure your GEMINI_API_KEY and QDRANT_API_KEY are valid
2. **Rate Limits**: The service respects free-tier limits (10 requests/minute per session)
3. **Network**: Ensure your server can connect to Qdrant Cloud and Google APIs
4. **Memory**: Large documents may require more memory for embedding

### Logging

Check the application logs for errors:

```bash
# For Docker deployment
docker-compose logs backend

# For development
# Check console output from uvicorn
```

### Performance

- Monitor response times in the logs
- Check that Qdrant vector search is performing well
- Verify that Gemini API calls are not timing out

## Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Support

If you encounter issues, please create an issue in the GitHub repository with:
- Detailed description of the problem
- Steps to reproduce
- Expected vs. actual behavior
- Environment information (OS, Python version, etc.)