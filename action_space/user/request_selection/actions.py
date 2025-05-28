from action_space.user.request_selection.animation import request_user_input as request
from cognition_layer.tools.translation.engine import translate
from ecm.tools.item_registry_v2 import ItemRegistry
from ecm.tools.item_registry_v2 import Storage

PKG_NAME = "request-selection"


# ========================= ACTIONS ======================
@ItemRegistry.register(type="action", package=PKG_NAME)
def request_user_input(message: str, options: list[str]) -> str:
    """
    Request user input with a message and options.
    Example: request_user_input("What would you like me to buy?", ["Ice Cream", "Pizza"])
    Example: request_user_input("I need you to provide the name of the file:", ["file1.txt", "file2.txt"])
    Your message must be short and concise, as it will sent to the user.
    Use me when you must do a choice in some selection.
    """
    language = Storage("RETURN_RESPONSE_CONFIG").get("language", "english")
    translated_message = translate(message, target_language=language)

    return request(translated_message, options)
