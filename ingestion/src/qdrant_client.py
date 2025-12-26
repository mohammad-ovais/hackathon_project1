import os
from typing import List, Dict, Any
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import PointStruct
from dotenv import load_dotenv
from .models import DocumentChunk

# Load environment variables
load_dotenv()

class QdrantService:
    """
    Service for interacting with Qdrant vector database
    """

    def __init__(self):
        # Initialize Qdrant client with connection pooling settings
        self.client = QdrantClient(
            url=os.getenv("QDRANT_URL"),
            api_key=os.getenv("QDRANT_API_KEY"),
            # Connection pooling and performance settings
            timeout=30,  # 30 second timeout
            retry_on_timeout=True,
            http2=True,  # Enable HTTP/2 for better performance
            # Additional connection settings for pooling
            **{
                "grpc_options": {
                    "grpc.keepalive_time_ms": 30000,
                    "grpc.keepalive_timeout_ms": 5000,
                    "grpc.http2.max_pings_without_data": 0,
                    "grpc.http2.min_time_between_pings_ms": 10000,
                    "grpc.http2.min_ping_interval_without_data_ms": 300000,
                }
            } if hasattr(QdrantClient, 'grpc_options') else {}
        )
        self.collection_name = os.getenv("QDRANT_COLLECTION_NAME", "textbook_chunks")

    def create_collection(self, vector_size: int = 768):
        """
        Create a collection in Qdrant for storing document chunks
        """
        try:
            # Check if collection already exists
            self.client.get_collection(self.collection_name)
            print(f"Collection '{self.collection_name}' already exists")
        except:
            # Create new collection
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(
                    size=vector_size,  # Default size for Gemini embeddings
                    distance=models.Distance.COSINE
                )
            )
            print(f"Created collection '{self.collection_name}'")

    def store_chunks(self, chunks: List[DocumentChunk]):
        """
        Store document chunks in Qdrant
        """
        points = []
        for chunk in chunks:
            # Create a point for Qdrant
            point = PointStruct(
                id=chunk.id,
                vector=chunk.embedding,
                payload={
                    "content": chunk.content,
                    "metadata": {
                        "chapter": chunk.metadata.chapter,
                        "section": chunk.metadata.section,
                        "page_url": chunk.metadata.page_url,
                        "source_file": chunk.metadata.source_file,
                        "token_count": chunk.metadata.token_count,
                        "position": chunk.metadata.position
                    },
                    "created_at": chunk.created_at.isoformat()
                }
            )
            points.append(point)

        # Upload points to Qdrant
        self.client.upsert(
            collection_name=self.collection_name,
            points=points
        )
        print(f"Stored {len(chunks)} chunks in Qdrant")

    def search_chunks(self, query_vector: List[float], limit: int = 3) -> List[Dict[str, Any]]:
        """
        Search for similar chunks in Qdrant
        """
        results = self.client.search(
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

    def delete_collection(self):
        """
        Delete the collection (useful for testing/reindexing)
        """
        try:
            self.client.delete_collection(self.collection_name)
            print(f"Deleted collection '{self.collection_name}'")
        except Exception as e:
            print(f"Error deleting collection: {e}")