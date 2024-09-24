import action_space.experimental.mouse.actions # noqa
import action_space.experimental.screenshot.actions  # noqa
import action_space.experimental.wait_for.actions # noqa
from ecm.tools.item_registry_v2 import ItemRegistry

if __name__ == "__main__":

    registry = ItemRegistry()
    registry.load_all()
    registry.summary()
