from cognition_layer.tools.voice.engine import play
from cognition_layer.tools.voice.engine import text2speech
from ecm.tools.item_registry_v2 import ItemRegistry

PKG_NAME = "return-response"


# ========================= ACTIONS ======================
@ItemRegistry.register(type="action", package=PKG_NAME)
def send_response_to_user(message: str) -> str:
    """
    Sends/returns a message response to the user.
    Example: send_response_to_user("The capital of France is Paris.")
    Example: send_response_to_user("The text requested is...")
    """
    speech = text2speech(message)
    play(speech)
    return "Successfully sent response to user."
