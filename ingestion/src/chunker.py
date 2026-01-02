import re
from typing import List, Tuple
from .models import DocumentChunk, ChunkMetadata


class TextChunker:
    """
    Service for chunking text into smaller segments with overlap
    """

    def __init__(self, chunk_size: int = 500, overlap: int = 50):
        self.chunk_size = chunk_size
        self.overlap = overlap
        

    def estimate_tokens(self, text: str) -> int:
        """
        simple & safe token estimate
        """
        return len(text.split())

    def chunk_text(self, text: str, metadata: ChunkMetadata) -> List[DocumentChunk]:
        """
        Split text into chunks with specified size and overlap
        """
        # Split text into sentences to avoid cutting in the middle of sentences
        sentences = re.split(r'[.!?]+', text)
        sentences = [s.strip() + '.' for s in sentences if s.strip()]

        chunks = []
        current_chunk = ""
        current_tokens = 0
        chunk_start_idx = 0

        for i, sentence in enumerate(sentences):
            sentence_tokens = self.estimate_tokens(sentence)

            # If adding this sentence would exceed chunk size
            if current_tokens + sentence_tokens > self.chunk_size and current_chunk:
                # Create a chunk with the current content
                chunk = self._create_chunk(current_chunk, metadata, chunk_start_idx)
                chunks.append(chunk)

                # Start a new chunk with overlap
                overlap_text = self._get_overlap_text(chunks, sentences, i)
                current_chunk = overlap_text + " " + sentence
                current_tokens = self.estimate_tokens(current_chunk)
                chunk_start_idx = max(0, len(" ".join(sentences[:i])) - len(overlap_text))
            else:
                # Add sentence to current chunk
                if current_chunk:
                    current_chunk += " " + sentence
                else:
                    current_chunk = sentence
                current_tokens += sentence_tokens

            # If we've reached the end and have content, create a final chunk
            if i == len(sentences) - 1 and current_chunk.strip():
                chunk = self._create_chunk(current_chunk, metadata, chunk_start_idx)
                chunks.append(chunk)

        return chunks

    def _get_overlap_text(self, existing_chunks: List[DocumentChunk], sentences: List[str], current_idx: int) -> str:
        """
        Get text from the end of the previous chunk for overlap
        """
        if not existing_chunks:
            return ""

        # Get the last few sentences from the previous chunk to use as overlap
        overlap_sentences = []
        tokens_counted = 0

        # Go backwards through sentences until we reach the overlap token count
        for i in range(current_idx - 1, max(0, current_idx - 10), -1):  # Look back at most 10 sentences
            sent_tokens = self.estimate_tokens(sentences[i])
            if tokens_counted + sent_tokens > self.overlap:
                break
            overlap_sentences.insert(0, sentences[i])
            tokens_counted += sent_tokens

        return " ".join(overlap_sentences)

    def _create_chunk(self, content: str, metadata: ChunkMetadata, start_idx: int) -> DocumentChunk:
        """
        Create a DocumentChunk with the given content and metadata
        """
        from uuid import uuid4
        from datetime import datetime

        return DocumentChunk(
            id=str(uuid4()),
            content=content,
            metadata=metadata.copy(update={"token_count": self.estimate_tokens(content), "position": start_idx}),
            created_at=datetime.now()
        )

    def chunk_file(self, file_path: str, metadata_template: ChunkMetadata) -> List[DocumentChunk]:
        """
        Read a file and chunk its content
        """
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()

        # Update metadata with source file info
        updated_metadata = metadata_template.copy(update={"source_file": file_path})
        return self.chunk_text(content, updated_metadata)