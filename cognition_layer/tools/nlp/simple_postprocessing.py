import re
import unicodedata
from typing import Callable
from typing import Literal
from typing import TypeVar

_T = TypeVar("_T")


def match(
    word: str,
    possible_elements: list[_T],
    key: Callable[[_T], str] = lambda word: str(word),
) -> tuple[_T, Literal["exact", "partial", "invalid"]]:
    """Matches a word with a list of possible elements."""

    keyword = sanitize_word(word)

    for element in possible_elements:
        if keyword == sanitize_word(key(element)):
            return element, "exact"

    for element in possible_elements:
        if keyword in sanitize_word(key(element)):
            return element, "partial"

    return None, "invalid"


def sanitize_word(word: str) -> str:
    """Sanitizes a word by removing unwanted characters."""
    word = unicodedata.normalize("NFD", word)
    word = word.encode("ascii", "ignore").decode("utf-8")
    word = re.sub(r"[^a-zA-Z0-9\s]", "", word)
    word = re.sub(r"\s+", " ", word).strip().lower()
    return word
