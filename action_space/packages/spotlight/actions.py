from action_space.packages.spotlight.app import spotlight as _spotlight
from ecm.tools.item_registry_v2 import ItemRegistry

PKG_NAME = "spotlight"
DURATION = 1.5


# ========================= ACTIONS ======================
@ItemRegistry.register(type="action", package=PKG_NAME)
def show_in_screen(x: int, y: int, target_radius: int) -> str:
    """
    Use it to highlight a specific area on the screen.
    Use it always that you must show, indicate, point or explain something in the screen.
    Example for highlighting pixel x=100, y=200: spotlight(100, 200, 200)
    For obtaining the coordinates please use vision tools.
    Target radius is the radius of the circle.
    Use 40 for icons or buttons, and values around 400 for medium elements.
    For window sized use 800 or more.
    Also use it for answering questions such as "Where is...", "Can you show me...", etc.
    """
    _spotlight(
        x=x,
        y=y,
        target_radius=target_radius,
        decay=target_radius,
        duration=DURATION,
    )
