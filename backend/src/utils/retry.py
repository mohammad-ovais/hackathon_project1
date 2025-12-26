import time
import random
from functools import wraps
from typing import Callable, Type, Any


def retry_with_backoff(
    max_retries: int = 3,
    base_delay: float = 1.0,
    max_delay: float = 60.0,
    backoff_factor: float = 2.0,
    exceptions: tuple = (Exception,)
):
    """
    Decorator for retrying functions with exponential backoff and jitter
    """
    def decorator(func: Callable) -> Callable:
        @wraps(func)
        def wrapper(*args, **kwargs) -> Any:
            last_exception = None

            for attempt in range(max_retries + 1):  # +1 to include the initial attempt
                try:
                    return func(*args, **kwargs)
                except exceptions as e:
                    last_exception = e

                    if attempt == max_retries:
                        # No more retries left
                        break

                    # Calculate delay with exponential backoff and jitter
                    delay = min(base_delay * (backoff_factor ** attempt), max_delay)
                    jitter = random.uniform(0, delay * 0.1)  # Add up to 10% jitter
                    total_delay = delay + jitter

                    print(f"Attempt {attempt + 1} failed: {e}. Retrying in {total_delay:.2f} seconds...")
                    time.sleep(total_delay)

            # If we've exhausted all retries, raise the last exception
            raise last_exception

        return wrapper
    return decorator


class RetryHandler:
    """
    A class to handle retry logic for API calls
    """

    @staticmethod
    @retry_with_backoff(
        max_retries=3,
        base_delay=1.0,
        max_delay=30.0,
        exceptions=(Exception,)
    )
    def call_with_retry(func: Callable, *args, **kwargs) -> Any:
        """
        Call a function with retry logic
        """
        return func(*args, **kwargs)

    @staticmethod
    def call_with_retry_and_validation(
        func: Callable,
        validation_func: Callable,
        max_retries: int = 3,
        base_delay: float = 1.0,
        max_delay: float = 30.0,
        exceptions: tuple = (Exception,)
    ) -> Any:
        """
        Call a function with retry logic and validation of the result
        """
        last_exception = None

        for attempt in range(max_retries + 1):
            try:
                result = func()

                # Validate the result
                if validation_func(result):
                    return result
                else:
                    raise ValueError(f"Validation failed for result: {result}")

            except exceptions as e:
                last_exception = e

                if attempt == max_retries:
                    # No more retries left
                    break

                # Calculate delay with exponential backoff and jitter
                delay = min(base_delay * (2.0 ** attempt), max_delay)
                jitter = random.uniform(0, delay * 0.1)  # Add up to 10% jitter
                total_delay = delay + jitter

                print(f"Attempt {attempt + 1} failed: {e}. Retrying in {total_delay:.2f} seconds...")
                time.sleep(total_delay)

        # If we've exhausted all retries, raise the last exception
        raise last_exception