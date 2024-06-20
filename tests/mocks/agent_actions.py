from ecm.tools.registry import ItemRegistry


@ItemRegistry.register_function
def click(place: str) -> None:
    """Clicks on the specified place. Use me as the unique tool for clicking in screen."""
    print("Clicked on ", place)


@ItemRegistry.register_function
def type(text: str) -> None:
    """Types the passed text. Use me always that you have to type some text."""
    print("Typing: ", text)


@ItemRegistry.register_function
def press(key: str) -> None:
    """Presses the passed key. Use me as an alternative to type."""
    print("Typing: ", key)
