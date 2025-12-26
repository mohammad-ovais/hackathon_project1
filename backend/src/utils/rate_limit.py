import time
from typing import Dict
from datetime import datetime, timedelta
from ..models import RateLimitRecord


class InMemoryRateLimiter:
    """
    In-memory rate limiter to enforce API limits per session
    """
    def __init__(self, requests_per_minute: int = 10, window_minutes: int = 1):
        self.requests_per_minute = requests_per_minute
        self.window_seconds = window_minutes * 60
        self.storage: Dict[str, RateLimitRecord] = {}

    def is_allowed(self, session_id: str) -> bool:
        """
        Check if a request is allowed for the given session
        """
        current_time = datetime.now()
        record = self.storage.get(session_id)

        if not record:
            # First request from this session
            self.storage[session_id] = RateLimitRecord(
                session_id=session_id,
                request_count=1,
                window_start=current_time,
                last_request=current_time
            )
            return True

        # Check if we're still in the same window
        time_since_window_start = (current_time - record.window_start).total_seconds()

        if time_since_window_start >= self.window_seconds:
            # Reset the window
            self.storage[session_id] = RateLimitRecord(
                session_id=session_id,
                request_count=1,
                window_start=current_time,
                last_request=current_time
            )
            return True

        # Check if we've exceeded the limit
        if record.request_count >= self.requests_per_minute:
            return False

        # Increment the request count
        record.request_count += 1
        record.last_request = current_time
        return True

    def get_reset_time(self, session_id: str) -> int:
        """
        Get the timestamp when the rate limit window resets
        """
        record = self.storage.get(session_id)
        if not record:
            return int((datetime.now() + timedelta(seconds=self.window_seconds)).timestamp())

        time_since_window_start = (datetime.now() - record.window_start).total_seconds()
        remaining_time = max(0, self.window_seconds - time_since_window_start)
        return int((datetime.now() + timedelta(seconds=remaining_time)).timestamp())

    def get_remaining_requests(self, session_id: str) -> int:
        """
        Get the number of remaining requests for this session in the current window
        """
        record = self.storage.get(session_id)
        if not record:
            return self.requests_per_minute

        time_since_window_start = (datetime.now() - record.window_start).total_seconds()

        if time_since_window_start >= self.window_seconds:
            return self.requests_per_minute

        return max(0, self.requests_per_minute - record.request_count)


# Global rate limiter instance
rate_limiter = InMemoryRateLimiter()