from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from dotenv import load_dotenv
import os

# Load environment variables
load_dotenv()

# Import API routers
from src.api import create_api_router

# Initialize the application
app = FastAPI(
    title="RAG Chatbot API",
    description="API for RAG-based chatbot for Physical AI & Humanoid Robotics textbook",
    version="1.0.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=[" https://mohammad-ovais.github.io/hackathon_project1/"],  # In production, specify exact origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include API routes
api_router = create_api_router()
app.include_router(api_router, prefix="/api")

@app.get("/")
def read_root():
    return {"message": "RAG Chatbot API is running!"}

@app.get("/health")
def health_check():
    return {"status": "healthy", "timestamp": "2025-12-26T00:00:00Z"}