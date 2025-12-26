"""
Unit tests for the RAG Service
"""
import unittest
from unittest.mock import Mock, patch, MagicMock
from src.services.rag_service import RAGService
from src.models import QueryRequest, QueryResponse, Citation, SourceChunk


class TestRAGService(unittest.TestCase):
    """Test cases for the RAGService class"""

    def setUp(self):
        """Set up test fixtures before each test method."""
        with patch('google.generativeai.configure'), \
             patch('qdrant_client.QdrantClient'), \
             patch('os.getenv', side_effect=lambda x, default=None: {
                 'GEMINI_API_KEY': 'fake-api-key',
                 'QDRANT_URL': 'http://localhost:6333',
                 'QDRANT_API_KEY': 'fake-qdrant-key',
                 'QDRANT_COLLECTION_NAME': 'test-collection'
             }.get(x, default)):
            self.rag_service = RAGService()

    @patch('google.generativeai.embed_content')
    def test_generate_embedding(self, mock_embed_content):
        """Test the generate_embedding method"""
        # Arrange
        mock_embed_content.return_value = {'embedding': [0.1, 0.2, 0.3]}

        # Act
        result = self.rag_service.generate_embedding("test query")

        # Assert
        self.assertEqual(result, [0.1, 0.2, 0.3])
        mock_embed_content.assert_called_once()

    @patch.object(RAGService, 'retrieve_chunks')
    @patch.object(RAGService, 'generate_response')
    @patch.object(RAGService, 'generate_citations')
    @patch.object(RAGService, 'create_source_chunks')
    def test_query_method(self, mock_create_sources, mock_gen_citations,
                         mock_gen_response, mock_retrieve_chunks):
        """Test the query method end-to-end"""
        # Arrange
        mock_retrieve_chunks.return_value = [
            {
                "id": "test-id",
                "content": "test content",
                "metadata": {
                    "chapter": "Chapter 1",
                    "section": "Section 1.1",
                    "page_url": "/docs/chapter1",
                    "source_file": "chapter1.md",
                    "token_count": 10,
                    "position": 0
                },
                "score": 0.9
            }
        ]
        mock_gen_response.return_value = "Test response"
        mock_gen_citations.return_value = [
            Citation(
                chapter="Chapter 1",
                section="Section 1.1",
                page_url="/docs/chapter1",
                text_snippet="test content",
                confidence=0.9
            )
        ]
        mock_create_sources.return_value = [
            SourceChunk(
                content="test content",
                metadata=MagicMock(),  # Mock metadata
                relevance_score=0.9
            )
        ]

        request = QueryRequest(
            query="What is Physical AI?",
            session_id="test-session-123",
            include_citations=True
        )

        # Act
        response = self.rag_service.query(request)

        # Assert
        self.assertIsInstance(response, QueryResponse)
        self.assertEqual(response.answer, "Test response")
        self.assertEqual(len(response.citations), 1)
        self.assertEqual(len(response.sources), 1)
        self.assertGreater(response.response_time, 0)

    def test_generate_citations(self):
        """Test the generate_citations method"""
        # Arrange
        chunks = [
            {
                "content": "Test content",
                "metadata": {
                    "chapter": "Chapter 1",
                    "section": "Section 1.1",
                    "page_url": "/docs/chapter1",
                    "source_file": "chapter1.md",
                    "token_count": 10,
                    "position": 0
                },
                "score": 0.9
            }
        ]

        # Act
        citations = self.rag_service.generate_citations(chunks)

        # Assert
        self.assertEqual(len(citations), 1)
        self.assertEqual(citations[0].chapter, "Chapter 1")
        self.assertEqual(citations[0].section, "Section 1.1")
        self.assertEqual(citations[0].page_url, "/docs/chapter1")
        self.assertEqual(citations[0].confidence, 0.9)

    @patch('google.generativeai.GenerativeModel')
    def test_generate_response(self, mock_model_class):
        """Test the generate_response method"""
        # Arrange
        mock_model_instance = Mock()
        mock_model_instance.generate_content.return_value = Mock(text="Generated response")
        mock_model_class.return_value = mock_model_instance

        chunks = [
            {
                "content": "Test context content",
                "metadata": {
                    "chapter": "Chapter 1",
                    "section": "Section 1.1",
                    "page_url": "/docs/chapter1",
                    "source_file": "chapter1.md",
                    "token_count": 10,
                    "position": 0
                },
                "score": 0.9
            }
        ]

        # Act
        response = self.rag_service.generate_response("Test query", chunks)

        # Assert
        self.assertEqual(response, "Generated response")
        mock_model_instance.generate_content.assert_called_once()

    def test_create_source_chunks(self):
        """Test the create_source_chunks method"""
        # Arrange
        chunks = [
            {
                "content": "Test content",
                "metadata": {
                    "chapter": "Chapter 1",
                    "section": "Section 1.1",
                    "page_url": "/docs/chapter1",
                    "source_file": "chapter1.md",
                    "token_count": 10,
                    "position": 0
                },
                "score": 0.9
            }
        ]

        # Act
        source_chunks = self.rag_service.create_source_chunks(chunks)

        # Assert
        self.assertEqual(len(source_chunks), 1)
        self.assertEqual(source_chunks[0].content, "Test content")
        self.assertEqual(source_chunks[0].relevance_score, 0.9)


if __name__ == '__main__':
    unittest.main()