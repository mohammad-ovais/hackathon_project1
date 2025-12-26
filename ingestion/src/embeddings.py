import os
import google.generativeai as genai
from typing import List
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Import the retry utility - we'll implement a simple retry mechanism here
import time
import random
from functools import wraps


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
    def decorator(func):
        @wraps(func)
        def wrapper(*args, **kwargs):
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

class EmbeddingService:
    """
    Service for generating embeddings using Gemini API
    """

    def __init__(self):
        # Configure Gemini API
        api_key = os.getenv("GEMINI_API_KEY")
        if not api_key:
            raise ValueError("GEMINI_API_KEY environment variable is required")

        genai.configure(api_key=api_key)
        self.model = "models/text-embedding-004"

    @retry_with_backoff(
        max_retries=3,
        base_delay=1.0,
        max_delay=30.0,
        exceptions=(Exception,)
    )
    def generate_embedding(self, text: str) -> List[float]:
        """
        Generate embedding for a single text with retry logic
        """
        response = genai.embed_content(
            model=self.model,
            content=text,
            task_type="RETRIEVAL_DOCUMENT"
        )
        return response['embedding']

    def generate_embeddings_batch(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for a batch of texts
        """
        embeddings = []
        for text in texts:
            try:
                # Use the individual retry-protected method
                embedding = self.generate_embedding(text)
                embeddings.append(embedding)
            except Exception as e:
                print(f"Failed to generate embedding for text '{text[:50]}...', error: {e}")
                # Return an empty embedding as fallback
                embeddings.append([])
        return embeddings

    def get_embedding_dimension(self) -> int:
        """
        Get the dimension of the embeddings (for Qdrant collection setup)
        """
        # Generate a test embedding to determine dimensions
        test_embedding = self.generate_embedding("test")
        return len(test_embedding) if test_embedding else 768  # Default fallback