from fastapi import APIRouter
from . import health, chat

def create_api_router():
    """
    Create and configure the main API router
    """
    api_router = APIRouter()

    # Include all API sub-routers
    api_router.include_router(health.router)
    api_router.include_router(chat.router)

    return api_router

__all__ = ["create_api_router"]