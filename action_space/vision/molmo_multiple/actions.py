from action_space.tools.image import load_image
from action_space.vision.molmo_multiple.request import search_points_on_image
from ecm.tools.item_registry_v2 import ItemRegistry

PKG_NAME = "molmo_element_finder_multiple_actions"


# ========================= ACTIONS ======================
@ItemRegistry.require_dependencies("screenshot")
@ItemRegistry.register(type="action", package=PKG_NAME)
def obtain_coords(entity: str) -> str:
    """
    Uses vision tools to obtain coordinates of a specific entity on the screen.
    Don't use this function for texts, instead use text-based functions. This tool is for visual entities
    such as buttons or icons.
    Example for obtaining coordinates of a button: obtain_coords("Edge close button")
    It can return multiple coordinates if the entity is not unique.
    Please handle it by selecting the most appropriate or retrying clicks on each one.
    """
    image = load_image(ItemRegistry.get("screenshot", type="tool").content())
    try:
        coords = search_points_on_image(entity, image)
    except ValueError:
        return f"Could not find any point for the prompt '{entity}'"
    return f"The following coordinates were found: {coords}"
