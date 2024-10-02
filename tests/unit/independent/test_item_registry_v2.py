from ecm.tools.item_registry_v2 import ItemRegistry
from ecm.tools.item_registry_v2 import Storage


def test_flush():
    @ItemRegistry.register(type="action", package="mouse")
    def func_1():
        """Im a test."""

    ItemRegistry.flush()
    registry = ItemRegistry()
    registry.load_all()

    assert len(registry.actions.values()) == 0


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
    ItemRegistry.flush()


def test_access():
    @ItemRegistry.register()
    def action_1():
        """Im a test."""
        ...

    ItemRegistry().load_all()

    assert "action_1" in list(ItemRegistry().actions.keys())
    assert len(ItemRegistry().actions.keys()) == 1

    @ItemRegistry.register(type="tool")
    def tool_1():
        """Im a test."""
        ...

    ItemRegistry().load_all()
    assert "tool_1" in ItemRegistry().tools.keys()
    assert len(ItemRegistry().tools.keys()) == 1
    ItemRegistry.flush()


def invalidation():

    target = []

    @ItemRegistry.register(type="action")
    def func_1():
        """Im a test."""
        target.append("action failed")

    @ItemRegistry.register(type="tool")
    def func_2():
        """Im a test."""
        target.append("tool failed")

    ItemRegistry().invalidate()
    default_reg = ItemRegistry()
    default_reg.actions["func_1"].content()
    default_reg.tools["func_2"].content()

    assert len(target) == 0


def test_storage():
    storage = Storage()
    storage_2 = Storage()
    storage_diff = Storage("different")

    storage["test"] = "result"

    assert storage is storage_2
    assert storage["test"] == "result" and storage_2["test"] == "result"
    assert "test" not in storage_diff.keys()
