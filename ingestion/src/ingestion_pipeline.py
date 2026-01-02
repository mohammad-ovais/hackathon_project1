import os
import sys
from typing import List
from dotenv import load_dotenv

# Add the parent directory to the path so we can import our modules
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from .chunker import TextChunker
from .embeddings import EmbeddingService
from .qdrant_client import QdrantService
from .models import DocumentChunk, ChunkMetadata

# Load environment variables
load_dotenv()

class IngestionPipeline:
    """
    Main pipeline for processing textbook content into vector database
    """

    def __init__(self, chunk_size: int = 500, overlap: int = 50):
        self.chunker = TextChunker(chunk_size=chunk_size, overlap=overlap)
        self.embedding_service = EmbeddingService()
        self.qdrant_service = QdrantService()

    def process_file(self, file_path: str, chapter: str, section: str, page_url: str) -> int:
        """
        Process a single file and store it in the vector database
        """
        print(f"Processing file: {file_path}")

        # Create metadata template
        metadata = ChunkMetadata(
            chapter=chapter,
            section=section,
            page_url=page_url,
            source_file=file_path,
            token_count=0,  # Will be updated during chunking
            position=0
        )

        # Read and chunk the file
        chunks = self.chunker.chunk_file(file_path, metadata)
        print(f"Created {len(chunks)} chunks from {file_path}")

        # Generate embeddings for each chunk
        for chunk in chunks:
            embedding = self.embedding_service.generate_embedding(chunk.content)
            chunk.embedding = embedding

        # Store chunks in Qdrant
        self.qdrant_service.store_chunks(chunks)
        return len(chunks)

    def process_directory(self, directory_path: str, base_url: str = "/docs/") -> int:
        """
        Process all text files in a directory
        """
        import glob
        import os

        total_chunks = 0
        # Find all markdown files in the directory
        md_files = glob.glob(os.path.join(directory_path, "**/*.md"), recursive=True)
        md_files += glob.glob(os.path.join(directory_path, "**/*.mdx"), recursive=True)

        print(f"Found {len(md_files)} markdown files to process")

        # Create the Qdrant collection
        embedding_size = self.embedding_service.get_embedding_dimension()
        self.qdrant_service.create_collection(vector_size=embedding_size)

        for file_path in md_files:
            # Create a simple URL based on file path
            relative_path = os.path.relpath(file_path, directory_path)
            page_url = base_url + relative_path.replace('\\', '/').replace('.md', '').replace('.mdx', '')

            # Extract chapter/section info from path
            path_parts = relative_path.split(os.sep)
            chapter = path_parts[-2] if len(path_parts) > 1 else "Introduction"
            filename = os.path.basename(relative_path)
            section = os.path.splitext(filename)[0]

            chunks_count = self.process_file(file_path, chapter, section, page_url)
            total_chunks += chunks_count

        return total_chunks

    def run(self, source_path: str, chunk_size: int = 500, overlap: int = 50):
        """
        Run the full ingestion pipeline
        """
        print("Starting ingestion pipeline...")

        # Create Qdrant collection
        embedding_size = self.embedding_service.get_embedding_dimension()
        self.qdrant_service.create_collection(vector_size=embedding_size)

        if os.path.isfile(source_path):
            # Process a single file
            chunks_processed = self.process_file(
                source_path,
                chapter="General",
                section=os.path.basename(source_path),
                page_url="/docs/" + os.path.basename(source_path)
            )
        elif os.path.isdir(source_path):
            # Process a directory
            chunks_processed = self.process_directory(source_path)
        else:
            raise ValueError(f"Source path does not exist: {source_path}")

        print(f"Ingestion pipeline completed! Processed {chunks_processed} chunks.")
        return {
            "status": "completed",
            "chunks_processed": chunks_processed,
            "metadata": {
                "source_path": source_path,
                "chunk_size": chunk_size,
                "overlap": overlap
            }
        }


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Ingestion pipeline for RAG chatbot")
    parser.add_argument("--source-path", type=str, required=True, help="Path to source documents")
    parser.add_argument("--chunk-size", type=int, default=500, help="Size of text chunks (default: 500)")
    parser.add_argument("--overlap", type=int, default=50, help="Overlap between chunks (default: 50)")

    args = parser.parse_args()

    pipeline = IngestionPipeline(chunk_size=args.chunk_size, overlap=args.overlap)
    result = pipeline.run(args.source_path, args.chunk_size, args.overlap)
    print(result)