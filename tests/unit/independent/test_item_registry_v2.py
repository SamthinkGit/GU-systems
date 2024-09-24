from ecm.tools.item_registry_v2 import ItemRegistry


def test_different_workspaces():
    @ItemRegistry.register(type="action", package="mouse")
    def func_1():
        """Im a test."""

    @ItemRegistry.register(type="action", package="mouse")
    def func_2():
        """Im a test2."""

    @ItemRegistry.register(type="action")
    def func_3():
        """Im a test3."""

    @ItemRegistry.register(type="tool")
    def func_4():
        """Im a tool."""

    registry = ItemRegistry()
    registry2 = ItemRegistry()
    assert id(registry) == id(registry2)

    super_registry = ItemRegistry(name="super-registry")
    assert id(super_registry) != id(registry2)
