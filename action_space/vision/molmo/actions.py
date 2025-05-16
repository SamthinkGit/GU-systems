from action_space.mouse.molmo_based.word2click import _find_point_on_screen
from ecm.tools.item_registry_v2 import ItemRegistry

PKG_NAME = "molmo_vision_actions"


# ========================= ACTIONS ======================
@ItemRegistry.register(type="action", package=PKG_NAME)
def obtain_coords(entity: str) -> str:
    """
    Uses vision tools to obtain coordinates of a specific entity on the screen.
    Example for obtaining coordinates of a button: obtain_coords("Edge close button")
    """
    try:
        coords = _find_point_on_screen(prompt=entity, location="fullscreen")
    except ValueError:
        return f"Could not find any point for the prompt '{entity}'"
    return f"x={coords[0]}, y={coords[1]}"
