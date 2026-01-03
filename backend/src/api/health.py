from fastapi import APIRouter
from typing import Dict, Any
import time
from datetime import datetime
import os
import google.generativeai as genai
from qdrant_client import QdrantClient

router = APIRouter()

@router.get("/health")
async def health_check() -> Dict[str, Any]:
    """
    Health check endpoint to verify the service is running
    """
    # Check Qdrant connection
    qdrant_status = "unknown"
    try:
        qdrant_client = QdrantClient(
            url=os.getenv("QDRANT_URL"),
            api_key=os.getenv("QDRANT_API_KEY")
        )
        # Try to get collection info to verify connection
        collection_name = os.getenv("QDRANT_COLLECTION_NAME", "textbook_chunks")
        qdrant_client.get_collection(collection_name)
        qdrant_status = "healthy"
    except Exception:
        qdrant_status = "unhealthy"

    # Check Gemini connection
    gemini_status = "unknown"
    try:
        api_key = os.getenv("GEMINI_API_KEY")
        if api_key:
            genai.configure(api_key=api_key)
            # Try a simple model call to verify connection
            model = genai.GenerativeModel('models/gemini-1.5-flash')
            gemini_status = "healthy"
        else:
            gemini_status = "unhealthy"
    except Exception:
        gemini_status = "unhealthy"

    return {
        "status": "healthy",
        "timestamp": datetime.now().isoformat(),
        "services": {
            "qdrant": qdrant_status,
            "gemini": gemini_status
        }
    }