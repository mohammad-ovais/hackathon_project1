import signal
import time
from functools import wraps
from typing import Any, Callable
import logging

logger = logging.getLogger(__name__)

class TimeoutError(Exception):
    """Exception raised when a function times out"""
    pass

def timeout_handler(signum, frame):
    """Handler for timeout signal"""
    raise TimeoutError("Function call timed out")

def timeout(seconds: int):
    """
    Decorator to timeout a function after specified seconds
    Note: This only works on Unix systems due to use of signal
    """
    def decorator(func: Callable) -> Callable:
        @wraps(func)
        def wrapper(*args, **kwargs) -> Any:
            # Only use signal-based timeout on Unix systems
            try:
                old_handler = signal.signal(signal.SIGALRM, timeout_handler)
                signal.alarm(seconds)
                try:
                    result = func(*args, **kwargs)
                    return result
                finally:
                    signal.alarm(0)  # Cancel the alarm
                    signal.signal(signal.SIGALRM, old_handler)  # Restore old handler
            except AttributeError:
                # On Windows, use time-based timeout instead
                start_time = time.time()
                result = func(*args, **kwargs)
                if time.time() - start_time > seconds:
                    raise TimeoutError(f"Function call timed out after {seconds} seconds")
                return result
        return wrapper
    return decorator


class GracefulDegradationHandler:
    """
    Handler for implementing graceful degradation when APIs are slow
    """

    @staticmethod
    def with_degradation(
        fallback_response: Any = None,
        timeout_seconds: int = 10,
        degrade_on_timeout: bool = True,
        degrade_on_error: bool = True
    ):
        """
        Decorator to implement graceful degradation
        """
        def decorator(func: Callable) -> Callable:
            @wraps(func)
            def wrapper(*args, **kwargs) -> Any:
                start_time = time.time()

                try:
                    # Attempt to call the function
                    result = func(*args, **kwargs)

                    # Check if the call took too long
                    execution_time = time.time() - start_time
                    if execution_time > timeout_seconds and degrade_on_timeout:
                        logger.warning(f"Function {func.__name__} took {execution_time:.2f}s, exceeding timeout of {timeout_seconds}s")

                    return result

                except Exception as e:
                    logger.error(f"Error in function {func.__name__}: {str(e)}")

                    if degrade_on_error:
                        logger.info(f"Returning fallback response for {func.__name__}")
                        return fallback_response
                    else:
                        raise  # Re-raise the exception

            return wrapper
        return decorator

    @staticmethod
    def with_timeout_and_fallback(
        func: Callable,
        timeout_seconds: int = 10,
        fallback_func: Callable = None,
        *args,
        **kwargs
    ) -> Any:
        """
        Call a function with timeout and fallback
        """
        start_time = time.time()

        try:
            # Attempt to call the function
            result = func(*args, **kwargs)

            # Check if the call took too long
            execution_time = time.time() - start_time
            if execution_time > timeout_seconds:
                logger.warning(f"Function took {execution_time:.2f}s, exceeding timeout of {timeout_seconds}s")
                if fallback_func:
                    logger.info("Using fallback function")
                    return fallback_func(*args, **kwargs)

            return result

        except Exception as e:
            logger.error(f"Error in function: {str(e)}")
            if fallback_func:
                logger.info("Using fallback function")
                return fallback_func(*args, **kwargs)
            else:
                return None


# Global instance
degradation_handler = GracefulDegradationHandler()