"""
Response Validation Module for the RAG Chatbot
Ensures that responses are properly grounded in the documentation to prevent hallucinations.
"""

import logging
from typing import List, Dict, Any, Tuple
from sentence_transformers import util
import torch

from .rag_chatbot import DocumentChunk

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class ResponseValidator:
    """
    Validates that chatbot responses are properly grounded in the source documentation
    to prevent hallucinations.
    """

    def __init__(self, threshold: float = 0.8):
        """
        Initialize the response validator.

        Args:
            threshold: Minimum similarity threshold for validation (0.0-1.0)
        """
        self.threshold = threshold
        # In a real implementation, you might load a sentence transformer model here
        # For now, we'll implement basic validation techniques

    def validate_response_grounding(self,
                                  query: str,
                                  response: str,
                                  retrieved_chunks: List[Dict[str, Any]]) -> Tuple[bool, List[str]]:
        """
        Validate that a response is properly grounded in the retrieved chunks.

        Args:
            query: Original user query
            response: Generated response from the chatbot
            retrieved_chunks: List of chunks retrieved by the RAG system

        Returns:
            Tuple of (is_valid, list_of_issues)
        """
        issues = []

        # Check if response contains fallback message when no relevant info was found
        if not retrieved_chunks and "not available in the provided documentation" in response.lower():
            return True, []

        # Check if response contains information that can be traced back to retrieved chunks
        if retrieved_chunks:
            # Check if response content is supported by retrieved chunks
            supported, unsupported_parts = self._check_content_support(response, retrieved_chunks)

            if not supported:
                issues.extend(unsupported_parts)
                return False, issues

        # Additional validation checks can be added here
        # For example: fact-checking against retrieved content, checking for contradictions, etc.

        return len(issues) == 0, issues

    def _check_content_support(self, response: str, retrieved_chunks: List[Dict[str, Any]]) -> Tuple[bool, List[str]]:
        """
        Check if the response content is supported by the retrieved chunks.

        Args:
            response: Generated response
            retrieved_chunks: List of retrieved chunks

        Returns:
            Tuple of (is_supported, list_of_unsupported_parts)
        """
        unsupported_parts = []

        # Split response into sentences for more granular validation
        sentences = self._split_into_sentences(response)

        for sentence in sentences:
            if not self._sentence_is_supported(sentence, retrieved_chunks):
                unsupported_parts.append(sentence.strip())

        return len(unsupported_parts) == 0, unsupported_parts

    def _split_into_sentences(self, text: str) -> List[str]:
        """
        Split text into sentences.

        Args:
            text: Text to split

        Returns:
            List of sentences
        """
        import re
        # Simple sentence splitting using common sentence delimiters
        sentences = re.split(r'[.!?]+', text)
        # Filter out empty strings and whitespace-only strings
        return [s for s in sentences if s.strip()]

    def _sentence_is_supported(self, sentence: str, retrieved_chunks: List[Dict[str, Any]]) -> bool:
        """
        Check if a sentence is supported by the retrieved chunks.

        Args:
            sentence: Sentence to check
            retrieved_chunks: List of retrieved chunks

        Returns:
            True if sentence is supported, False otherwise
        """
        if not sentence.strip():
            return True  # Empty sentences are considered supported

        # Check if the sentence content appears in any of the retrieved chunks
        sentence_lower = sentence.lower().strip()

        for chunk in retrieved_chunks:
            chunk_content = chunk.get('content', '').lower()
            # Check if the sentence or significant parts of it appear in the chunk
            if self._text_contains_sentence(chunk_content, sentence_lower):
                return True

        # If sentence doesn't directly appear, check for semantic similarity
        return self._check_semantic_similarity(sentence_lower, retrieved_chunks)

    def _text_contains_sentence(self, text: str, sentence: str) -> bool:
        """
        Check if text contains the sentence (with some tolerance for variations).

        Args:
            text: Text to search in
            sentence: Sentence to search for

        Returns:
            True if text contains sentence, False otherwise
        """
        if not sentence:
            return True

        # Simple containment check with some preprocessing
        # Remove extra whitespace and check for containment
        text_clean = ' '.join(text.split())
        sentence_clean = ' '.join(sentence.split())

        return sentence_clean in text_clean

    def _check_semantic_similarity(self, sentence: str, retrieved_chunks: List[Dict[str, Any]], threshold: float = 0.5) -> bool:
        """
        Check semantic similarity between sentence and retrieved chunks.

        Args:
            sentence: Sentence to check
            retrieved_chunks: List of retrieved chunks
            threshold: Similarity threshold

        Returns:
            True if semantic similarity is above threshold, False otherwise
        """
        # In a real implementation, this would use embeddings to compute semantic similarity
        # For now, we'll use a simple keyword overlap approach
        sentence_words = set(sentence.lower().split())

        for chunk in retrieved_chunks:
            chunk_content = chunk.get('content', '')
            chunk_words = set(chunk_content.lower().split())

            # Calculate overlap ratio
            intersection = sentence_words.intersection(chunk_words)
            if intersection:
                overlap_ratio = len(intersection) / len(sentence_words)
                if overlap_ratio >= threshold:
                    return True

        return False

    def validate_factual_accuracy(self, response: str, retrieved_chunks: List[Dict[str, Any]]) -> Tuple[bool, List[str]]:
        """
        Validate factual accuracy of the response against retrieved chunks.

        Args:
            response: Generated response
            retrieved_chunks: List of retrieved chunks

        Returns:
            Tuple of (is_accurate, list_of_factual_issues)
        """
        factual_issues = []

        # Extract claims from the response
        claims = self._extract_claims(response)

        for claim in claims:
            if not self._claim_is_supported(claim, retrieved_chunks):
                factual_issues.append(f"Claim not supported by documentation: '{claim}'")

        return len(factual_issues) == 0, factual_issues

    def _extract_claims(self, text: str) -> List[str]:
        """
        Extract factual claims from text.

        Args:
            text: Text to extract claims from

        Returns:
            List of extracted claims
        """
        # This is a simplified implementation
        # In a real system, you'd use NLP techniques to identify claims
        sentences = self._split_into_sentences(text)
        # Filter out questions and other non-claim sentences
        claims = []
        for sentence in sentences:
            sentence = sentence.strip()
            if sentence and not sentence.endswith('?'):
                # Additional filtering could be applied here
                claims.append(sentence)
        return claims

    def _claim_is_supported(self, claim: str, retrieved_chunks: List[Dict[str, Any]]) -> bool:
        """
        Check if a claim is supported by the retrieved chunks.

        Args:
            claim: Claim to verify
            retrieved_chunks: List of retrieved chunks

        Returns:
            True if claim is supported, False otherwise
        """
        # In a real implementation, this would use more sophisticated fact-checking
        # For now, we'll use the same approach as sentence support
        return self._sentence_is_supported(claim, retrieved_chunks)

    def validate_response_completeness(self, query: str, response: str, retrieved_chunks: List[Dict[str, Any]]) -> Tuple[bool, str]:
        """
        Validate that the response adequately addresses the query based on retrieved information.

        Args:
            query: Original user query
            response: Generated response
            retrieved_chunks: List of retrieved chunks

        Returns:
            Tuple of (is_complete, completion_feedback)
        """
        # Check if response addresses key terms from the query
        query_terms = set(query.lower().split())
        response_lower = response.lower()

        # Count how many query terms are addressed in the response
        addressed_terms = [term for term in query_terms if term in response_lower]

        if len(addressed_terms) < len(query_terms) * 0.5:  # At least 50% of terms should be addressed
            missing_terms = [term for term in query_terms if term not in response_lower]
            return False, f"Response does not adequately address all aspects of the query. Missing: {', '.join(missing_terms[:5])}"  # Limit feedback length

        return True, "Response adequately addresses the query"

    def comprehensive_validation(self, query: str, response: str, retrieved_chunks: List[Dict[str, Any]]) -> Dict[str, Any]:
        """
        Perform comprehensive validation of the response.

        Args:
            query: Original user query
            response: Generated response
            retrieved_chunks: List of retrieved chunks

        Returns:
            Dictionary with validation results
        """
        grounding_valid, grounding_issues = self.validate_response_grounding(query, response, retrieved_chunks)
        factual_valid, factual_issues = self.validate_factual_accuracy(response, retrieved_chunks)
        completeness_valid, completeness_feedback = self.validate_response_completeness(query, response, retrieved_chunks)

        is_valid = grounding_valid and factual_valid and completeness_valid

        return {
            "is_valid": is_valid,
            "grounding_valid": grounding_valid,
            "factual_valid": factual_valid,
            "completeness_valid": completeness_valid,
            "issues": {
                "grounding": grounding_issues,
                "factual": factual_issues,
                "completeness": completeness_feedback
            },
            "confidence_score": self._calculate_confidence_score(
                grounding_valid, factual_valid, completeness_valid,
                len(grounding_issues), len(factual_issues)
            )
        }

    def _calculate_confidence_score(self,
                                  grounding_valid: bool,
                                  factual_valid: bool,
                                  completeness_valid: bool,
                                  grounding_issue_count: int,
                                  factual_issue_count: int) -> float:
        """
        Calculate a confidence score based on validation results.

        Args:
            grounding_valid: Whether grounding validation passed
            factual_valid: Whether factual validation passed
            completeness_valid: Whether completeness validation passed
            grounding_issue_count: Number of grounding issues
            factual_issue_count: Number of factual issues

        Returns:
            Confidence score between 0 and 1
        """
        score = 1.0  # Start with perfect score

        # Deduct points for validation failures
        if not grounding_valid:
            score -= 0.4  # Grounding is most important
        if not factual_valid:
            score -= 0.3  # Factual accuracy is very important
        if not completeness_valid:
            score -= 0.2  # Completeness is important
        if grounding_issue_count > 0:
            score -= min(0.1 * grounding_issue_count, 0.3)  # Up to 0.3 deduction for grounding issues
        if factual_issue_count > 0:
            score -= min(0.1 * factual_issue_count, 0.3)  # Up to 0.3 deduction for factual issues

        return max(0.0, score)  # Ensure score doesn't go below 0