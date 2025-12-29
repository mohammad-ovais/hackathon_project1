from pydantic import BaseModel
from typing import List, Optional
from datetime import datetime


class QueryRequest(BaseModel):
    """
    Structure for incoming query requests
    """
    query: str
    selected_text: Optional[str] = None
    session_id: str
    include_citations: bool = True


class SourceChunk(BaseModel):
    """
    Details about chunks used in response generation
    """
    content: str
    metadata: 'ChunkMetadata'
    relevance_score: float


class Citation(BaseModel):
    """
    Reference to specific parts of the textbook
    """
    chapter: str
    section: str
    page_url: str
    text_snippet: str
    confidence: float


class QueryResponse(BaseModel):
    """
    Structure for response to query requests
    """
    answer: str
    citations: List[Citation]
    sources: List[SourceChunk]
    response_time: float


