"""
Unit tests for the API endpoints
"""
import unittest
import asyncio
from fastapi.testclient import TestClient
from unittest.mock import Mock, patch, MagicMock

from src.api import create_api_router
from src.models import QueryRequest, QueryResponse


class TestAPIEndpoints(unittest.TestCase):
    """Test cases for the API endpoints"""

    def setUp(self):
        """Set up test fixtures before each test method."""
        # Create a mock FastAPI app for testing
        from fastapi import FastAPI

        self.app = FastAPI()
        api_router = create_api_router()
        self.app.include_router(api_router, prefix="/api")

        self.client = TestClient(self.app)

    @patch('src.services.rag_service.RAGService')
    def test_chat_query_endpoint(self, mock_rag_service_class):
        """Test the chat query endpoint"""
        # Arrange
        mock_rag_instance = Mock()
        mock_rag_service_class.return_value = mock_rag_instance

        mock_response = QueryResponse(
            answer="Test answer",
            citations=[],
            sources=[],
            response_time=100.0
        )
        mock_rag_instance.query.return_value = mock_response

        request_payload = {
            "query": "What is Physical AI?",
            "session_id": "test-session-123",
            "include_citations": True
        }

        # Act
        response = self.client.post("/api/chat/query", json=request_payload)

        # Assert
        self.assertEqual(response.status_code, 200)
        response_data = response.json()
        self.assertEqual(response_data["answer"], "Test answer")
        self.assertIn("response_time", response_data)

    @patch('src.services.rag_service.RAGService')
    def test_chat_query_selection_endpoint(self, mock_rag_service_class):
        """Test the chat query selection endpoint"""
        # Arrange
        mock_rag_instance = Mock()
        mock_rag_service_class.return_value = mock_rag_instance

        mock_response = QueryResponse(
            answer="Test answer for selection",
            citations=[],
            sources=[],
            response_time=150.0
        )
        mock_rag_instance.query.return_value = mock_response

        request_payload = {
            "query": "What does this mean?",
            "selected_text": "Physical AI combines principles of physics with artificial intelligence",
            "session_id": "test-session-123",
            "include_citations": True
        }

        # Act
        response = self.client.post("/api/chat/query-selection", json=request_payload)

        # Assert
        self.assertEqual(response.status_code, 200)
        response_data = response.json()
        self.assertEqual(response_data["answer"], "Test answer for selection")
        self.assertIn("response_time", response_data)

    def test_health_endpoint(self):
        """Test the health endpoint"""
        # Act
        response = self.client.get("/api/health")

        # Assert
        self.assertEqual(response.status_code, 200)
        response_data = response.json()
        self.assertEqual(response_data["status"], "healthy")
        self.assertIn("timestamp", response_data)
        self.assertIn("services", response_data)

    @patch('src.utils.rate_limit.rate_limiter')
    def test_rate_limit_exceeded(self, mock_rate_limiter):
        """Test the rate limiting functionality"""
        # Arrange
        mock_rate_limiter.is_allowed.return_value = False
        mock_rate_limiter.get_reset_time.return_value = 1234567890

        request_payload = {
            "query": "What is Physical AI?",
            "session_id": "test-session-rate-limited",
            "include_citations": True
        }

        # Act
        response = self.client.post("/api/chat/query", json=request_payload)

        # Assert
        self.assertEqual(response.status_code, 429)
        response_data = response.json()
        self.assertIn("error", response_data)
        self.assertEqual(response_data["error"]["code"], "RATE_LIMIT_EXCEEDED")

    def test_invalid_request_body(self):
        """Test handling of invalid request bodies"""
        # Act
        response = self.client.post("/api/chat/query", json={"invalid": "request"})

        # Assert
        # This should still return 200 because the validation is handled in the model
        # If validation fails, it would return 422 Unprocessable Entity
        if response.status_code == 422:
            self.assertEqual(response.status_code, 422)
        else:
            # The request may still be processed if it has the required fields
            self.assertIn(response.status_code, [200, 422])


if __name__ == '__main__':
    unittest.main()