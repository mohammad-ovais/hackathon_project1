from fastapi import APIRouter, HTTPException, Request
from fastapi.responses import JSONResponse
import time
import logging
from typing import Dict, Any

from ..models import QueryRequest
from ..services.rag_service import RAGService
from ..utils.rate_limit import rate_limiter, InMemoryRateLimiter

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

router = APIRouter()
rag_service = RAGService()

@router.post("/chat/query")
async def query_endpoint(request: QueryRequest):
    """
    Handle general questions about the textbook content with comprehensive error handling
    """
    try:
        # Validate request
        if not request.query or not request.query.strip():
            raise HTTPException(
                status_code=400,
                detail={
                    "error": {
                        "code": "INVALID_REQUEST",
                        "message": "Query is required and cannot be empty"
                    }
                }
            )

        if not request.session_id:
            raise HTTPException(
                status_code=400,
                detail={
                    "error": {
                        "code": "INVALID_REQUEST",
                        "message": "Session ID is required"
                    }
                }
            )

        # Check rate limit
        if not rate_limiter.is_allowed(request.session_id):
            logger.warning(f"Rate limit exceeded for session: {request.session_id}")
            return JSONResponse(
                status_code=429,
                content={
                    "error": {
                        "code": "RATE_LIMIT_EXCEEDED",
                        "message": "Rate limit exceeded. Please try again later.",
                        "details": {
                            "reset_time": rate_limiter.get_reset_time(request.session_id),
                            "remaining_requests": 0
                        }
                    }
                },
                headers={
                    "X-RateLimit-Limit": str(rate_limiter.requests_per_minute),
                    "X-RateLimit-Remaining": "0",
                    "X-RateLimit-Reset": str(rate_limiter.get_reset_time(request.session_id))
                }
            )

        # Perform the query with error handling
        start_time = time.time()
        try:
            response = rag_service.query(request)
        except Exception as rag_error:
            logger.error(f"RAG service error for session {request.session_id}: {str(rag_error)}")
            # Log the full exception for debugging
            import traceback
            logger.error(f"Full traceback: {traceback.format_exc()}")

            # Return a graceful error response
            headers = {
                "X-RateLimit-Limit": str(rate_limiter.requests_per_minute),
                "X-RateLimit-Remaining": str(rate_limiter.get_remaining_requests(request.session_id)),
                "X-RateLimit-Reset": str(rate_limiter.get_reset_time(request.session_id))
            }

            return JSONResponse(
                status_code=503,  # Service unavailable
                content={
                    "error": {
                        "code": "SERVICE_UNAVAILABLE",
                        "message": "The AI service is temporarily unavailable. Please try again later.",
                        "details": "We're experiencing high demand or technical issues with our AI provider."
                    }
                },
                headers=headers
            )

        total_time = (time.time() - start_time) * 1000  # Convert to milliseconds

        # Add rate limit headers
        headers = {
            "X-RateLimit-Limit": str(rate_limiter.requests_per_minute),
            "X-RateLimit-Remaining": str(rate_limiter.get_remaining_requests(request.session_id)),
            "X-RateLimit-Reset": str(rate_limiter.get_reset_time(request.session_id))
        }

        return JSONResponse(
            content={
                "answer": response.answer,
                "citations": [c.dict() for c in response.citations],
                "sources": [s.dict() for s in response.sources],
                "response_time": total_time
            },
            headers=headers
        )
    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        logger.error(f"Unexpected error in query endpoint: {str(e)}")
        import traceback
        logger.error(f"Full traceback: {traceback.format_exc()}")

        # Add rate limit headers even for error responses
        headers = {
            "X-RateLimit-Limit": str(rate_limiter.requests_per_minute),
            "X-RateLimit-Remaining": str(rate_limiter.get_remaining_requests(request.session_id)),
            "X-RateLimit-Reset": str(rate_limiter.get_reset_time(request.session_id))
        }

        return JSONResponse(
            status_code=500,
            content={
                "error": {
                    "code": "INTERNAL_ERROR",
                    "message": "An internal server error occurred",
                    "details": "Please try again later or contact support if the issue persists"
                }
            },
            headers=headers
        )

@router.post("/chat/query-selection")
async def query_selection_endpoint(request: QueryRequest):
    """
    Handle questions specifically about selected text with comprehensive error handling
    """
    try:
        # Validate request
        if not request.query or not request.query.strip():
            raise HTTPException(
                status_code=400,
                detail={
                    "error": {
                        "code": "INVALID_REQUEST",
                        "message": "Query is required and cannot be empty"
                    }
                }
            )

        if not request.selected_text:
            raise HTTPException(
                status_code=400,
                detail={
                    "error": {
                        "code": "INVALID_REQUEST",
                        "message": "selected_text is required for this endpoint"
                    }
                }
            )

        if not request.session_id:
            raise HTTPException(
                status_code=400,
                detail={
                    "error": {
                        "code": "INVALID_REQUEST",
                        "message": "Session ID is required"
                    }
                }
            )

        # Check rate limit
        if not rate_limiter.is_allowed(request.session_id):
            logger.warning(f"Rate limit exceeded for session: {request.session_id}")
            return JSONResponse(
                status_code=429,
                content={
                    "error": {
                        "code": "RATE_LIMIT_EXCEEDED",
                        "message": "Rate limit exceeded. Please try again later.",
                        "details": {
                            "reset_time": rate_limiter.get_reset_time(request.session_id),
                            "remaining_requests": 0
                        }
                    }
                },
                headers={
                    "X-RateLimit-Limit": str(rate_limiter.requests_per_minute),
                    "X-RateLimit-Remaining": "0",
                    "X-RateLimit-Reset": str(rate_limiter.get_reset_time(request.session_id))
                }
            )

        # Perform the query (same as regular query, but with selected text context)
        start_time = time.time()
        try:
            response = rag_service.query(request)
        except Exception as rag_error:
            logger.error(f"RAG service error for selection query session {request.session_id}: {str(rag_error)}")
            # Log the full exception for debugging
            import traceback
            logger.error(f"Full traceback: {traceback.format_exc()}")

            # Return a graceful error response
            headers = {
                "X-RateLimit-Limit": str(rate_limiter.requests_per_minute),
                "X-RateLimit-Remaining": str(rate_limiter.get_remaining_requests(request.session_id)),
                "X-RateLimit-Reset": str(rate_limiter.get_reset_time(request.session_id))
            }

            return JSONResponse(
                status_code=503,  # Service unavailable
                content={
                    "error": {
                        "code": "SERVICE_UNAVAILABLE",
                        "message": "The AI service is temporarily unavailable. Please try again later.",
                        "details": "We're experiencing high demand or technical issues with our AI provider."
                    }
                },
                headers=headers
            )

        total_time = (time.time() - start_time) * 1000  # Convert to milliseconds

        # Add rate limit headers
        headers = {
            "X-RateLimit-Limit": str(rate_limiter.requests_per_minute),
            "X-RateLimit-Remaining": str(rate_limiter.get_remaining_requests(request.session_id)),
            "X-RateLimit-Reset": str(rate_limiter.get_reset_time(request.session_id))
        }

        return JSONResponse(
            content={
                "answer": response.answer,
                "citations": [c.dict() for c in response.citations],
                "sources": [s.dict() for s in response.sources],
                "response_time": total_time
            },
            headers=headers
        )
    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        logger.error(f"Unexpected error in query selection endpoint: {str(e)}")
        import traceback
        logger.error(f"Full traceback: {traceback.format_exc()}")

        # Add rate limit headers even for error responses
        headers = {
            "X-RateLimit-Limit": str(rate_limiter.requests_per_minute),
            "X-RateLimit-Remaining": str(rate_limiter.get_remaining_requests(request.session_id)),
            "X-RateLimit-Reset": str(rate_limiter.get_reset_time(request.session_id))
        }

        return JSONResponse(
            status_code=500,
            content={
                "error": {
                    "code": "INTERNAL_ERROR",
                    "message": "An internal server error occurred",
                    "details": "Please try again later or contact support if the issue persists"
                }
            },
            headers=headers
        )