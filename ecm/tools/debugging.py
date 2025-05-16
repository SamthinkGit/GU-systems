from ecm.tools.registry import ItemRegistry


def call_action(name: str, package: str, *args, **kwargs):
    registry = ItemRegistry()
    registry.autoload(package)
    action = registry.get(name, type="action")
    if action is None:
        raise ValueError(f"Action {name} not found in package {package}")
    return action.content(*args, **kwargs)
