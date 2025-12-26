import time
import functools
from typing import Callable, Any
from datetime import datetime
import logging

# Set up logging for performance metrics
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class PerformanceMonitor:
    """
    Performance monitoring utility for tracking execution times and resource usage
    """

    def __init__(self):
        self.metrics = {}

    def record_metric(self, name: str, value: float, unit: str = "ms"):
        """
        Record a performance metric
        """
        if name not in self.metrics:
            self.metrics[name] = []
        self.metrics[name].append({
            'value': value,
            'unit': unit,
            'timestamp': datetime.now().isoformat()
        })

    def get_average(self, name: str) -> float:
        """
        Get the average value for a metric
        """
        if name not in self.metrics or not self.metrics[name]:
            return 0.0
        values = [m['value'] for m in self.metrics[name]]
        return sum(values) / len(values)

    def get_last(self, name: str) -> float:
        """
        Get the last recorded value for a metric
        """
        if name not in self.metrics or not self.metrics[name]:
            return 0.0
        return self.metrics[name][-1]['value']

    def log_performance(self, name: str, value: float, unit: str = "ms"):
        """
        Log performance metric to the logger
        """
        logger.info(f"PERFORMANCE - {name}: {value:.2f} {unit}")
        self.record_metric(name, value, unit)


def measure_time(func: Callable) -> Callable:
    """
    Decorator to measure execution time of a function
    """
    @functools.wraps(func)
    def wrapper(*args, **kwargs) -> Any:
        start_time = time.perf_counter()
        try:
            result = func(*args, **kwargs)
            return result
        finally:
            end_time = time.perf_counter()
            execution_time = (end_time - start_time) * 1000  # Convert to milliseconds
            logger.info(f"Execution time for {func.__name__}: {execution_time:.2f} ms")
    return wrapper


def monitor_performance(name: str):
    """
    Decorator to monitor performance of a function with a custom name
    """
    def decorator(func: Callable) -> Callable:
        @functools.wraps(func)
        def wrapper(*args, **kwargs) -> Any:
            start_time = time.perf_counter()
            try:
                result = func(*args, **kwargs)
                return result
            finally:
                end_time = time.perf_counter()
                execution_time = (end_time - start_time) * 1000  # Convert to milliseconds
                monitor = PerformanceMonitor()
                monitor.log_performance(name, execution_time)
        return wrapper
    return decorator


# Global performance monitor instance
perf_monitor = PerformanceMonitor()