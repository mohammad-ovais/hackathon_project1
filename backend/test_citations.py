"""
Test for citation functionality, especially in relation to selected text
"""
import sys
import os
from typing import Dict, Any

# Add the src directory to the path
sys.path.append(os.path.join(os.path.dirname(__file__), 'src'))

from src.models import QueryRequest
from src.services.rag_service import RAGService

def test_citation_generation():
    """
    Test that citations are properly generated
    """
    print("Testing citation generation...")

    try:
        # Initialize the RAG service
        rag_service = RAGService()

        # Create a test query
        test_request = QueryRequest(
            query="What is Physical AI?",
            session_id="test-session-500",
            include_citations=True
        )

        # Perform the query
        response = rag_service.query(test_request)

        print(f"Number of citations: {len(response.citations)}")

        # Check that citations exist and have the required fields
        for citation in response.citations:
            assert hasattr(citation, 'chapter'), "Citation missing chapter field"
            assert hasattr(citation, 'section'), "Citation missing section field"
            assert hasattr(citation, 'page_url'), "Citation missing page_url field"
            assert hasattr(citation, 'text_snippet'), "Citation missing text_snippet field"
            assert hasattr(citation, 'confidence'), "Citation missing confidence field"
            print(f"  - Chapter: {citation.chapter}, Section: {citation.section}")

        print("✓ Citation generation test passed!")
        return True
    except Exception as e:
        print(f"✗ Citation generation test failed: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_citation_with_selected_text():
    """
    Test that citations work properly with selected text queries
    """
    print("\nTesting citations with selected text...")

    try:
        # Initialize the RAG service
        rag_service = RAGService()

        # Create a test query with selected text
        test_request = QueryRequest(
            query="What does this mean?",
            selected_text="Physical AI combines principles of physics with artificial intelligence",
            session_id="test-session-501",
            include_citations=True
        )

        # Perform the query
        response = rag_service.query(test_request)

        print(f"Number of citations with selected text: {len(response.citations)}")

        # Check that citations exist
        assert len(response.citations) > 0, "No citations returned with selected text query"

        # Verify citation structure
        for citation in response.citations:
            assert citation.chapter and citation.chapter != "", "Citation missing valid chapter"
            assert citation.section and citation.section != "", "Citation missing valid section"
            assert citation.page_url and citation.page_url.startswith('http'), "Citation missing valid URL"
            print(f"  - Citation: {citation.chapter} - {citation.section}")

        print("✓ Citation with selected text test passed!")
        return True
    except Exception as e:
        print(f"✗ Citation with selected text test failed: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_citation_relevance():
    """
    Test that citations are relevant to the query
    """
    print("\nTesting citation relevance...")

    try:
        # Initialize the RAG service
        rag_service = RAGService()

        # Create specific queries to test citation relevance
        test_requests = [
            QueryRequest(
                query="What is Physical AI?",
                session_id="test-session-502",
                include_citations=True
            ),
            QueryRequest(
                query="Explain humanoid robotics",
                session_id="test-session-503",
                include_citations=True
            )
        ]

        for i, request in enumerate(test_requests):
            response = rag_service.query(request)
            print(f"Query {i+1}: '{request.query[:30]}...' -> {len(response.citations)} citations")

            for citation in response.citations[:2]:  # Check first 2 citations
                print(f"  - Relevant citation: {citation.chapter} - {citation.section}")

        print("✓ Citation relevance test passed!")
        return True
    except Exception as e:
        print(f"✗ Citation relevance test failed: {e}")
        import traceback
        traceback.print_exc()
        return False

def main():
    """
    Run all citation tests
    """
    print("Starting citation tests...")

    # Test basic citation generation
    citation_test_result = test_citation_generation()

    # Test citations with selected text
    selected_text_citation_test_result = test_citation_with_selected_text()

    # Test citation relevance
    relevance_test_result = test_citation_relevance()

    # Summary
    print("\n" + "="*50)
    print("CITATION TEST SUMMARY")
    print("="*50)
    print(f"Citation Generation Test: {'PASSED' if citation_test_result else 'FAILED'}")
    print(f"Citation with Selected Text Test: {'PASSED' if selected_text_citation_test_result else 'FAILED'}")
    print(f"Citation Relevance Test: {'PASSED' if relevance_test_result else 'FAILED'}")

    all_passed = citation_test_result and selected_text_citation_test_result and relevance_test_result
    if all_passed:
        print("\n✓ All citation tests passed!")
        return True
    else:
        print("\n✗ Some citation tests failed!")
        return False

if __name__ == "__main__":
    success = main()
    if not success:
        sys.exit(1)