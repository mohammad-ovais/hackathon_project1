import uuid
from datetime import datetime, timedelta
from typing import Dict, Optional
from ..models import Session, ChatMessage


class InMemorySessionManager:
    """
    In-memory session manager to handle chat sessions
    """
    def __init__(self, session_timeout_minutes: int = 30):
        self.session_timeout = timedelta(minutes=session_timeout)
        self.sessions: Dict[str, Session] = {}

    def create_session(self) -> str:
        """
        Create a new session and return the session ID
        """
        session_id = str(uuid.uuid4())
        session = Session(
            session_id=session_id,
            history=[],
            created_at=datetime.now(),
            last_activity=datetime.now()
        )
        self.sessions[session_id] = session
        return session_id

    def get_session(self, session_id: str) -> Optional[Session]:
        """
        Get a session by ID, returning None if it doesn't exist or has expired
        """
        session = self.sessions.get(session_id)
        if not session:
            return None

        # Check if session has expired
        if datetime.now() - session.last_activity > self.session_timeout:
            del self.sessions[session_id]
            return None

        return session

    def add_message_to_session(self, session_id: str, role: str, content: str) -> bool:
        """
        Add a message to a session
        """
        session = self.get_session(session_id)
        if not session:
            return False

        message = ChatMessage(
            role=role,
            content=content,
            timestamp=datetime.now()
        )
        session.history.append(message)
        session.last_activity = datetime.now()
        return True

    def clear_expired_sessions(self):
        """
        Remove all expired sessions from memory
        """
        current_time = datetime.now()
        expired_sessions = [
            session_id for session_id, session in self.sessions.items()
            if current_time - session.last_activity > self.session_timeout
        ]

        for session_id in expired_sessions:
            del self.sessions[session_id]

        return len(expired_sessions)


# Global session manager instance
session_manager = InMemorySessionManager()