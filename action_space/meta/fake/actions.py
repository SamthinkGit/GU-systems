from ecm.tools.item_registry_v2 import ItemRegistry

PKG_NAME = "meta-look"


@ItemRegistry.register(type="action", package=PKG_NAME)
def look_to_screen():
    """
    Request the user to send an screenshot of the screen with the current focused part of the screen.
    The next message of the user will be the screenshot.
    Please, make sure to describe relevant information that you see in the image since you only will have the image one during message.
    """  # noqa
    pass
