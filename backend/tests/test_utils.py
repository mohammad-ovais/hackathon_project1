"""
Unit tests for utility functions
"""
import unittest
from unittest.mock import Mock, patch
from datetime import datetime, timedelta
import time

from src.utils.rate_limit import InMemoryRateLimiter
from src.utils.session import InMemorySessionManager
from src.utils.cache import InMemoryCache


class TestUtils(unittest.TestCase):
    """Test cases for utility functions"""

    def test_rate_limiter_allows_requests(self):
        """Test that the rate limiter allows requests within limits"""
        # Arrange
        rate_limiter = InMemoryRateLimiter(requests_per_minute=2, window_minutes=1)

        # Act & Assert
        # First request should be allowed
        self.assertTrue(rate_limiter.is_allowed("session-1"))

        # Second request should be allowed
        self.assertTrue(rate_limiter.is_allowed("session-1"))

        # Third request should be denied (exceeds limit of 2 per minute)
        self.assertFalse(rate_limiter.is_allowed("session-1"))

    def test_rate_limiter_resets_after_window(self):
        """Test that the rate limiter resets after the time window"""
        # Arrange
        rate_limiter = InMemoryRateLimiter(requests_per_minute=1, window_minutes=1)

        # Use the one allowed request
        self.assertTrue(rate_limiter.is_allowed("session-2"))

        # Next request should be denied
        self.assertFalse(rate_limiter.is_allowed("session-2"))

        # Manually reset the window start time to simulate time passing
        rate_limiter.storage["session-2"].window_start = datetime.now() - timedelta(minutes=2)

        # Now the request should be allowed again
        self.assertTrue(rate_limiter.is_allowed("session-2"))

    def test_session_manager_creates_sessions(self):
        """Test that the session manager creates and manages sessions"""
        # Arrange
        session_manager = InMemorySessionManager(session_timeout_minutes=30)

        # Act
        session_id = session_manager.create_session()

        # Assert
        self.assertIsNotNone(session_id)
        self.assertTrue(len(session_id) > 0)

        # Get the created session
        session = session_manager.get_session(session_id)
        self.assertIsNotNone(session)
        self.assertEqual(session.session_id, session_id)
        self.assertEqual(len(session.history), 0)

    def test_session_manager_adds_messages(self):
        """Test that the session manager adds messages to sessions"""
        # Arrange
        session_manager = InMemorySessionManager(session_timeout_minutes=30)
        session_id = session_manager.create_session()

        # Act
        result = session_manager.add_message_to_session(session_id, "user", "Hello")

        # Assert
        self.assertTrue(result)

        # Get the session and check the message was added
        session = session_manager.get_session(session_id)
        self.assertEqual(len(session.history), 1)
        self.assertEqual(session.history[0].role, "user")
        self.assertEqual(session.history[0].content, "Hello")

    def test_session_manager_handles_expired_sessions(self):
        """Test that the session manager handles expired sessions"""
        # Arrange
        session_manager = InMemorySessionManager(session_timeout_minutes=0)  # Expire immediately
        session_id = session_manager.create_session()

        # Act
        session = session_manager.get_session(session_id)

        # Assert
        # The session should be expired and thus return None
        self.assertIsNone(session)

    def test_cache_basic_operations(self):
        """Test basic cache operations: set, get, delete"""
        # Arrange
        cache = InMemoryCache(default_ttl_seconds=60)

        # Act & Assert
        # Set a value
        cache.set("key1", "value1")

        # Get the value
        result = cache.get("key1")
        self.assertEqual(result, "value1")

        # Delete the value
        result = cache.delete("key1")
        self.assertTrue(result)

        # Try to get the deleted value
        result = cache.get("key1")
        self.assertIsNone(result)

    def test_cache_ttl_expiration(self):
        """Test that cache entries expire after TTL"""
        # Arrange
        cache = InMemoryCache(default_ttl_seconds=0.1)  # Very short TTL for testing

        # Act
        cache.set("expiring_key", "expiring_value")

        # Verify it exists initially
        result = cache.get("expiring_key")
        self.assertEqual(result, "expiring_value")

        # Wait for expiration
        time.sleep(0.2)  # Sleep longer than TTL

        # Try to get after expiration
        result = cache.get("expiring_key")
        self.assertIsNone(result)

    def test_cache_clear_expired(self):
        """Test clearing expired cache entries"""
        # Arrange
        cache = InMemoryCache(default_ttl_seconds=0.1)  # Very short TTL for testing

        # Set multiple values
        cache.set("key1", "value1")
        cache.set("key2", "value2")
        cache.set("key3", "value3")

        # Wait for expiration
        time.sleep(0.2)

        # Act
        removed_count = cache.clear_expired()

        # Assert
        self.assertEqual(removed_count, 3)  # All 3 should have expired
        self.assertEqual(cache.size(), 0)

    def test_cache_size_tracking(self):
        """Test that cache size is tracked correctly"""
        # Arrange
        cache = InMemoryCache(default_ttl_seconds=60)

        # Act & Assert
        self.assertEqual(cache.size(), 0)

        cache.set("key1", "value1")
        self.assertEqual(cache.size(), 1)

        cache.set("key2", "value2")
        self.assertEqual(cache.size(), 2)

        cache.delete("key1")
        self.assertEqual(cache.size(), 1)


if __name__ == '__main__':
    unittest.main()