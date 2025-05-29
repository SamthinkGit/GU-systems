from action_space.user.request_selection.animation import request_user_input as request
from cognition_layer.tools.translation.engine import translate
from ecm.tools.item_registry_v2 import ItemRegistry
from ecm.tools.item_registry_v2 import Storage

PKG_NAME = "request-selection"


# ========================= ACTIONS ======================
@ItemRegistry.register(type="action", package=PKG_NAME, labels=["enforce-host"])
def request_user_input(message: str, options: list[str]) -> str:
    """
    Request user input with a message and options.
    Example: request_user_input("What would you like me to buy?", ["Ice Cream", "Pizza"])
    Example: request_user_input("I need you to provide the name of the file:", ["file1.txt", "file2.txt"])
    Your message must be short and concise, as it will sent to the user.
    Use me when you must do a choice in some selection.
    """

    # This translation must be done in the host (Because it contains the llms)
    language = Storage("RETURN_RESPONSE_CONFIG").get("language", "english")
    translated_message = translate(message, target_language=language)

    # We use the item registry to ensure the gui is executed on the remote client.
    _request = ItemRegistry().get(name="_request_user_input_with_gui", type="tool")

    return _request.content(translated_message, options)


@ItemRegistry.register(type="tool", package=PKG_NAME)
def _request_user_input_with_gui(message: str, options: list[str]) -> str:
    """
    Internal tool to request user input.
    This is used by the action `request_user_input`.
    """
    return request(message, options)
