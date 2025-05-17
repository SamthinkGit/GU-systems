from ecm.tools.item_registry_v2 import ItemRegistry
from action_space.user.request_selection.animation import request_user_input as request

PKG_NAME = "request-selection"


# ========================= ACTIONS ======================
@ItemRegistry.register(type="action", package=PKG_NAME)
def request_user_input(message: str, options: list[str]) -> str:
    """
    Request user input with a message and options.
    Example: request_user_input("What would you like me to buy?", ["Ice Cream", "Pizza"])
    Ensure to match the language of the user.
    Use me when you must do a choice in some selection.
    """
    return request(message, options)
