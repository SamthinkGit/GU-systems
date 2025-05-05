from action_space.meta.cognition_state.state import _cognition_state_setter
from ecm.tools.item_registry_v2 import ItemRegistry

PKG_NAME = "meta"


@ItemRegistry.register(type="action", package=PKG_NAME, labels=["enforce-host"])
def set_value_on_cognition_state(key: str, value: str) -> None:
    """Sets a value in the cognition state. Use me for updating the cognition state. Example: set_value_on_cognition_state("example_field", "test_value")"""  # noqa
    _cognition_state_setter(key, value, "default")
