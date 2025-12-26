"""
Test for selected text query functionality
"""
import sys
import os
from typing import Dict, Any

# Add the src directory to the path
sys.path.append(os.path.join(os.path.dirname(__file__), 'src'))

from src.models import QueryRequest
from src.services.rag_service import RAGService

def test_selected_text_query():
    """
    Test the selected text query functionality
    """
    print("Testing selected text query functionality...")

    try:
        # Initialize the RAG service
        rag_service = RAGService()

        # Create a test query with selected text
        test_request = QueryRequest(
            query="What does this mean?",
            selected_text="Physical AI combines principles of physics with artificial intelligence",
            session_id="test-session-456",
            include_citations=True
        )

        # Perform the query
        print("Sending test query with selected text...")
        response = rag_service.query(test_request)

        print(f"Response received:")
        print(f"Answer: {response.answer[:200]}...")  # Print first 200 chars
        print(f"Number of citations: {len(response.citations)}")
        print(f"Number of sources: {len(response.sources)}")
        print(f"Response time: {response.response_time:.2f}ms")

        # Verify the response
        assert response.answer is not None and len(response.answer) > 0
        assert response.response_time > 0
        print("✓ Selected text query test passed!")

        return True
    except Exception as e:
        print(f"✗ Selected text query test failed: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_regular_vs_selected_text():
    """
    Compare responses between regular query and selected text query
    """
    print("\nComparing regular vs selected text queries...")

    try:
        rag_service = RAGService()

        # Regular query
        regular_request = QueryRequest(
            query="What is Physical AI?",
            session_id="test-session-789",
            include_citations=True
        )

        regular_response = rag_service.query(regular_request)

        # Selected text query
        selected_request = QueryRequest(
            query="What does this mean?",
            selected_text="Physical AI combines principles of physics with artificial intelligence",
            session_id="test-session-790",
            include_citations=True
        )

        selected_response = rag_service.query(selected_request)

        print("Regular query response length:", len(regular_response.answer))
        print("Selected text query response length:", len(selected_response.answer))
        print("Both queries completed successfully")
        print("✓ Regular vs selected text comparison test passed!")

        return True
    except Exception as e:
        print(f"✗ Regular vs selected text comparison test failed: {e}")
        import traceback
        traceback.print_exc()
        return False

def main():
    """
    Run all selected text query tests
    """
    print("Starting selected text query tests...")

    # Test the selected text functionality
    selected_text_test_result = test_selected_text_query()

    # Test comparison between regular and selected text queries
    comparison_test_result = test_regular_vs_selected_text()

    # Summary
    print("\n" + "="*60)
    print("SELECTED TEXT QUERY TEST SUMMARY")
    print("="*60)
    print(f"Selected Text Query Test: {'PASSED' if selected_text_test_result else 'FAILED'}")
    print(f"Comparison Test: {'PASSED' if comparison_test_result else 'FAILED'}")

    if selected_text_test_result and comparison_test_result:
        print("\n✓ All selected text query tests passed!")
        return True
    else:
        print("\n✗ Some selected text query tests failed!")
        return False

if __name__ == "__main__":
    success = main()
    if not success:
        sys.exit(1)