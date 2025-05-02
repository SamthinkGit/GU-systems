from difflib import SequenceMatcher


def word_similarity(a: str, b: str) -> float:
    """Calculates the similarity between two words using SequenceMatcher."""
    return SequenceMatcher(None, a, b).ratio()
