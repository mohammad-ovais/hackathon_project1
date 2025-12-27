---
title: RAG_Chatbot
emoji: 🤖
colorFrom: blue
colorTo: purple
sdk: docker
app_file: main.py
pinned: false
license: mit
---

# Physical AI RAG Chatbot - Backend API

FastAPI backend for RAG-based chatbot supporting Physical AI & Humanoid Robotics textbook.

## Tech Stack
- **FastAPI** - REST API framework
- **Google Gemini** - LLM & embeddings
- **Qdrant** - Vector database
- **Neon Postgres** - Relational database

## API Endpoints
- `GET /` - Health check
- `GET /docs` - Interactive API documentation
- `POST /api/chat` - Chat with RAG context
- `POST /api/ingest` - Add text to vector database
- `GET /api/collection/stats` - Collection statistics

## Environment Variables Required
- `GEMINI_API_KEY`
- `NEON_DATABASE_URL`
- `QDRANT_URL`
- `QDRANT_API_KEY`
- `QDRANT_COLLECTION_NAME`

## Deployment
Deployed on Hugging Face Spaces using Docker.
