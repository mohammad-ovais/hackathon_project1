import time
import logging
from typing import List, Dict, Any
from datetime import datetime
from ..models import QueryRequest, QueryResponse, Citation, SourceChunk, DocumentChunk
from ..utils.rate_limit import rate_limiter
from ..utils.cache import embeddings_cache
from ..utils.retry import retry_with_backoff, RetryHandler
from ..utils.performance import measure_time, monitor_performance, perf_monitor
from ..utils.timeout import timeout, TimeoutError, GracefulDegradationHandler
from qdrant_client import QdrantClient
from qdrant_client.http import models
import google.generativeai as genai
import os
from dotenv import load_dotenv

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Load environment variables
load_dotenv()

class RAGService:
    """
    Service for orchestrating the RAG (Retrieval-Augmented Generation) process
    """

    def __init__(self):
        #Initialize Qdrant client
        self.qdrant_client = QdrantClient(
            url=os.getenv("QDRANT_URL"),
            api_key=os.getenv("QDRANT_API_KEY"),
         timeout=30
        )

        self.collection_name = os.getenv("QDRANT_COLLECTION_NAME", "textbook_chunks")

        # Configure Gemini API
        api_key = os.getenv("GEMINI_API_KEY")
        if not api_key:
             raise ValueError("GEMINI_API_KEY environment variable is required")

        genai.configure(api_key=api_key)
        self.gemini_model = genai.GenerativeModel('gemini-2.0-flash')

    def generate_embedding(self, text: str) -> List[float]:
        """
        Generate embedding for a text using Gemini API with caching
        """
        # Create a cache key based on the text content
        cache_key = f"embedding:{hash(text)}"

        # Try to get from cache first
        cached_embedding = embeddings_cache.get(cache_key)
        if cached_embedding is not None:
            return cached_embedding

        try:
            response = genai.embed_content(
                model="models/text-embedding-004",
                content=text,
                task_type="retrieval_query"  # Use QUERY for search queries
            )
            embedding = response['embedding']

            # Cache the result for 10 minutes
            embeddings_cache.set(cache_key, embedding, ttl_seconds=600)

            return embedding
        except Exception as e:
            print(f"Error generating embedding: {e}")
            return []

    @retry_with_backoff(
        max_retries=3,
        base_delay=1.0,
        max_delay=10.0,
        exceptions=(Exception,)
    )
    def retrieve_chunks(self, query_vector: List[float], limit: int = 3) -> List[Dict[str, Any]]:
        """
        Retrieve relevant chunks from the vector database with retry logic
        """
        try:
            results = self.qdrant_client.search(
                collection_name=self.collection_name,
                query_vector=query_vector,
                limit=limit
            )

            # Format results
            formatted_results = []
            for result in results:
                formatted_results.append({
                    "id": result.id,
                    "content": result.payload["content"],
                    "metadata": result.payload["metadata"],
                    "score": result.score
                })

            return formatted_results
        except Exception as e:
            print(f"Error retrieving chunks: {e}")
            raise  # Re-raise to trigger retry

    @retry_with_backoff(
        max_retries=2,
        base_delay=1.0,
        max_delay=15.0,
        exceptions=(Exception,)
    )
    @GracefulDegradationHandler.with_degradation(
        fallback_response="I'm currently experiencing high load. Please try again in a moment.",
        timeout_seconds=25,
        degrade_on_timeout=True,
        degrade_on_error=True
    )
    def generate_response(self, query: str, context_chunks: List[Dict[str, Any]], selected_text: str = None) -> str:
        """
        Generate response using Gemini based on the query and context with retry logic and graceful degradation
        """
        # Build context from retrieved chunks
        context = ""
        for chunk in context_chunks:
            context += f"Context: {chunk['content']}\n\n"

        # Include selected text if provided
        if selected_text:
            query = f"Based on the selected text: '{selected_text}', {query.lower()}"

        # Create the prompt for Gemini
        prompt = f"""
        You are an assistant for the Physical AI & Humanoid Robotics textbook.
        Answer the user's question based only on the provided context from the textbook.
        If the answer is not in the context, say "I don't have enough information from the textbook to answer that question."

        Context:
        {context}

        Question: {query}

        Answer:
        """
        response = self.gemini_model.generate_content(prompt)
        return response.text

    def generate_citations(self, chunks: List[Dict[str, Any]]) -> List[Citation]:
        """
        Generate citations from the retrieved chunks
        """
        citations = []
        for chunk in chunks:
            metadata = chunk['metadata']
            citation = Citation(
                chapter=metadata['chapter'],
                section=metadata['section'],
                page_url=metadata['page_url'],
                text_snippet=chunk['content'][:100] + "..." if len(chunk['content']) > 100 else chunk['content'],
                confidence=chunk['score']  # Use the similarity score as confidence
            )
            citations.append(citation)

        return citations

    def create_source_chunks(self, chunks: List[Dict[str, Any]]) -> List[SourceChunk]:
        """
        Create source chunk objects from retrieved chunks
        """
        from ..models import ChunkMetadata  # Import here to avoid circular dependency

        source_chunks = []
        for chunk in chunks:
            metadata = chunk['metadata']
            chunk_metadata = ChunkMetadata(
                chapter=metadata['chapter'],
                section=metadata['section'],
                page_url=metadata['page_url'],
                source_file=metadata['source_file'],
                token_count=metadata['token_count'],
                position=metadata['position']
            )

            source_chunk = SourceChunk(
                content=chunk['content'],
                metadata=chunk_metadata,
                relevance_score=chunk['score']
            )
            source_chunks.append(source_chunk)

        return source_chunks

    @monitor_performance("rag_query_total_time")
    def query(self, request: QueryRequest) -> QueryResponse:
        """
        Main query method that orchestrates the RAG process with performance monitoring and privacy-safe logging
        """
        start_time = time.time()
        perf_start_time = time.perf_counter()

        # Log the start of the query with session info only (no user data)
        logger.info(f"Processing query for session {request.session_id[:8]}...")

        # Check rate limit
        if not rate_limiter.is_allowed(request.session_id):
            logger.warning(f"Rate limit exceeded for session {request.session_id[:8]}")
            raise Exception("Rate limit exceeded")

        # Generate embedding for the query
        embedding_start = time.perf_counter()
        query_vector = self.generate_embedding(request.query)
        embedding_time = (time.perf_counter() - embedding_start) * 1000
        perf_monitor.log_performance("embedding_generation_time", embedding_time)

        if not query_vector:
            logger.error(f"Failed to generate query embedding for session {request.session_id[:8]}")
            raise Exception("Failed to generate query embedding")

        # Retrieve relevant chunks
        retrieval_start = time.perf_counter()
        retrieved_chunks = self.retrieve_chunks(query_vector, limit=3)
        retrieval_time = (time.perf_counter() - retrieval_start) * 1000
        perf_monitor.log_performance("chunk_retrieval_time", retrieval_time)

        # Generate response
        generation_start = time.perf_counter()
        response_text = self.generate_response(
            request.query,
            retrieved_chunks,
            request.selected_text
        )
        generation_time = (time.perf_counter() - generation_start) * 1000
        perf_monitor.log_performance("response_generation_time", generation_time)

        # Generate citations and source chunks
        citation_start = time.perf_counter()
        citations = self.generate_citations(retrieved_chunks)
        citation_time = (time.perf_counter() - citation_start) * 1000
        perf_monitor.log_performance("citation_generation_time", citation_time)

        source_start = time.perf_counter()
        source_chunks = self.create_source_chunks(retrieved_chunks)
        source_time = (time.perf_counter() - source_start) * 1000
        perf_monitor.log_performance("source_chunk_generation_time", source_time)

        # Calculate response time
        total_response_time = (time.time() - start_time) * 1000  # Convert to milliseconds
        actual_execution_time = (time.perf_counter() - perf_start_time) * 1000

        # Log performance metrics
        perf_monitor.log_performance("rag_query_total_time", total_response_time)
        perf_monitor.log_performance("rag_query_execution_time", actual_execution_time)

        # Log successful completion with timing info only (no user data)
        logger.info(f"Query completed for session {request.session_id[:8]} in {total_response_time:.2f}ms")

        return QueryResponse(
            answer=response_text,
            citations=citations if request.include_citations else [],
            sources=source_chunks,
            response_time=total_response_time
        )