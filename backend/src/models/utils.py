from pydantic import BaseModel
from typing import List
from datetime import datetime


class RateLimitRecord(BaseModel):
    """
    Track API usage for rate limiting
    """
    session_id: str
    request_count: int
    window_start: datetime
    last_request: datetime


class ChatMessage(BaseModel):
    """
    Individual chat message in session history
    """
    role: str
    content: str
    timestamp: datetime


class Session(BaseModel):
    """
    Temporary session data (not persisted)
    """
    session_id: str
    history: List[ChatMessage]
    created_at: datetime
    last_activity: datetime