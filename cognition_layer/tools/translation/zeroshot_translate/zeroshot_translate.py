from langsmith import traceable

from cognition_layer.tools.mutable_llm import MutableChatLLM


@traceable(run_type="llm", name="ZeroShotTranslate")
def translate(text: str, target_language: str) -> str:
    """
    Translate the given text to the target language using a zero-shot translation model.

    Args:
        text (str): The text to be translated.
        target_language (str): The language code of the target language (e.g., 'en' for English, 'es' for Spanish).

    Returns:
        str: The translated text.
    """
    llm = MutableChatLLM(model="gpt-4.1-nano")
    response = llm.invoke(
        f"You are a translation model. Translate the following text to `{target_language}`: `{text}`",
    )
    return response.content
