from cognition_layer.tools.translation.engine import translate
from cognition_layer.tools.voice.engine import play
from cognition_layer.tools.voice.engine import text2speech
from ecm.tools.item_registry_v2 import ItemRegistry
from ecm.tools.item_registry_v2 import Storage

PKG_NAME = "return-response"

# Please select the


# ========================= ACTIONS ======================
@ItemRegistry.register(type="action", package=PKG_NAME, labels=["enforce-host"])
def send_response_to_user(message: str) -> str:
    """
    Sends/returns a message response to the user.
    Example: send_response_to_user("The capital of France is Paris.")
    Example: send_response_to_user("The text requested is...")
    Your message must be short and concise, as it will be read to the user.
    """
    language = Storage("RETURN_RESPONSE_CONFIG").get("language", "english")
    voice = Storage("RETURN_RESPONSE_CONFIG").get("voice", True)

    if voice:
        translated_message = translate(message, target_language=language)
        speech = text2speech(translated_message)
        play(speech)
    return "Successfully sent response to user."
