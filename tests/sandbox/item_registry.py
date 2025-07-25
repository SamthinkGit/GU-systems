from ecm.remote.server import EcmServer
from ecm.tools.item_registry_v2 import ItemRegistry

if __name__ == "__main__":

    registry = ItemRegistry()

    @registry.register()
    def action_1():
        """Im a test."""
        print("Called")

    registry = ItemRegistry()
    registry.load_all()
    registry.invalidate()
    EcmServer.wrap_item_registry(ItemRegistry())

    alt_registry = ItemRegistry("alt")
    alt_registry.load_all()

    ItemRegistry.summary()
