from action_space.experimental.wait_for.agent import DEFAULT_WAIT_FOR_AGENT
from ecm.tools.registry import ItemRegistry


@ItemRegistry.register_function
def wait_for(condition: str):
    """Waits for a given condition. Useful when you must wait for a page to load, an application to open, etc. Usage: Give a description of the condition to wait. E.g. wait_for('Photoshop App is fully open')""" # noqa
    DEFAULT_WAIT_FOR_AGENT.wait_for(condition)
