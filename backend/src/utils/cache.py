import time
from typing import Dict, Any, Optional
from datetime import datetime, timedelta
from functools import wraps


class InMemoryCache:
    """
    Simple in-memory cache with TTL (Time To Live)
    """
    def __init__(self, default_ttl_seconds: int = 300):  # 5 minutes default
        self.cache: Dict[str, Dict[str, Any]] = {}
        self.default_ttl = default_ttl_seconds

    def set(self, key: str, value: Any, ttl_seconds: Optional[int] = None) -> None:
        """
        Set a value in the cache with optional TTL
        """
        ttl = ttl_seconds if ttl_seconds is not None else self.default_ttl
        expiry = datetime.now() + timedelta(seconds=ttl)

        self.cache[key] = {
            'value': value,
            'expiry': expiry
        }

    def get(self, key: str) -> Optional[Any]:
        """
        Get a value from the cache, return None if not found or expired
        """
        if key not in self.cache:
            return None

        entry = self.cache[key]
        if datetime.now() > entry['expiry']:
            # Entry has expired, remove it
            del self.cache[key]
            return None

        return entry['value']

    def delete(self, key: str) -> bool:
        """
        Delete a key from the cache
        """
        if key in self.cache:
            del self.cache[key]
            return True
        return False

    def clear_expired(self) -> int:
        """
        Clear all expired entries and return count of removed entries
        """
        current_time = datetime.now()
        expired_keys = [
            key for key, entry in self.cache.items()
            if current_time > entry['expiry']
        ]

        for key in expired_keys:
            del self.cache[key]

        return len(expired_keys)

    def size(self) -> int:
        """
        Get the number of entries in the cache
        """
        # Clean up expired entries first
        self.clear_expired()
        return len(self.cache)


def cached(ttl_seconds: int = 300):
    """
    Decorator to cache function results
    """
    def decorator(func):
        cache = InMemoryCache(ttl_seconds)

        @wraps(func)
        def wrapper(*args, **kwargs):
            # Create a cache key from function name and arguments
            cache_key = f"{func.__name__}:{str(args)}:{str(sorted(kwargs.items()))}"

            # Try to get from cache first
            cached_result = cache.get(cache_key)
            if cached_result is not None:
                return cached_result

            # If not in cache, call the function and cache the result
            result = func(*args, **kwargs)
            cache.set(cache_key, result, ttl_seconds)
            return result

        # Add cache instance to the wrapper for manual management
        wrapper.cache = cache
        return wrapper
    return decorator


# Global cache instance for embeddings
embeddings_cache = InMemoryCache(default_ttl_seconds=600)  # 10 minutes for embeddings