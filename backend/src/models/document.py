from pydantic import BaseModel
from typing import List, Optional
from datetime import datetime


class ChunkMetadata(BaseModel):
    """
    Metadata for citations and source tracking
    """
    chapter: str
    section: str
    page_url: str
    source_file: str
    token_count: int
    position: int


class DocumentChunk(BaseModel):
    """
    Represents a chunk of textbook content with embeddings
    """
    id: str
    content: str
    embedding: List[float]
    metadata: ChunkMetadata
    created_at: datetime