import time

from ecm.tools.item_registry_v2 import ItemRegistry

PKG_NAME = "sleep"


# ========================= ACTIONS ======================
@ItemRegistry.register(type="action", package=PKG_NAME)
def sleep(seconds: int) -> str:
    """
    Sleep for a specified number of seconds.
    This action is useful for pausing until an app is open (generally 5-10s), or a page is loaded (3-6s)...
    Use it repeatedly if needed.
    Example: sleep(5)
    """
    time.sleep(seconds)
