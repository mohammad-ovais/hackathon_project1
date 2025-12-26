"""
Simple test script for the ingestion pipeline
This script creates sample content and tests the ingestion process
"""
import os
import sys
import tempfile
from pathlib import Path

# Add the src directory to the path
sys.path.append(os.path.join(os.path.dirname(__file__), 'src'))

from src.ingestion_pipeline import IngestionPipeline

def create_sample_content():
    """
    Create sample textbook content for testing
    """
    content = """
# Introduction to Physical AI and Humanoid Robotics

## What is Physical AI?

Physical AI combines principles of physics with artificial intelligence to create systems that can interact with the physical world. This field has gained significant attention in recent years due to advances in robotics and machine learning.

## Humanoid Robotics

Humanoid robots are robots with a human-like body structure. They are designed to mimic the movements and actions of humans. Key components include:

- Actuators for movement
- Sensors for perception
- Control systems for coordination
- Learning algorithms for adaptation

## Applications

Physical AI and humanoid robotics have numerous applications including:
- Healthcare assistance
- Manufacturing
- Research
- Entertainment
- Education

## Challenges

Despite significant progress, several challenges remain:
- Energy efficiency
- Real-time processing
- Human-robot interaction
- Safety and reliability
"""

    return content

def test_ingestion_pipeline():
    """
    Test the ingestion pipeline with sample content
    """
    print("Creating sample content for testing...")

    # Create a temporary file with sample content
    with tempfile.NamedTemporaryFile(mode='w', suffix='.md', delete=False) as temp_file:
        temp_file.write(create_sample_content())
        temp_file_path = temp_file.name

    try:
        print(f"Created temporary file: {temp_file_path}")

        # Initialize the ingestion pipeline
        pipeline = IngestionPipeline(chunk_size=500, overlap=50)

        print("Starting ingestion pipeline test...")

        # Process the temporary file
        result = pipeline.process_file(
            file_path=temp_file_path,
            chapter="Introduction",
            section="Test Section",
            page_url="/test/introduction"
        )

        print(f"Successfully processed {result} chunks from the sample content")

        return True
    except Exception as e:
        print(f"Error during ingestion test: {e}")
        import traceback
        traceback.print_exc()
        return False
    finally:
        # Clean up the temporary file
        if os.path.exists(temp_file_path):
            os.unlink(temp_file_path)
            print(f"Cleaned up temporary file: {temp_file_path}")

if __name__ == "__main__":
    print("Testing ingestion pipeline...")
    success = test_ingestion_pipeline()
    if success:
        print("Ingestion pipeline test completed successfully!")
    else:
        print("Ingestion pipeline test failed!")
        sys.exit(1)