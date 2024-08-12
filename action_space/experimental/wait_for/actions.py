from action_space.experimental.wait_for.agent import DEFAULT_WAIT_FOR_AGENT
from ecm.tools.registry import ItemRegistry


@ItemRegistry.register_function
def wait_for(condition: str):
    DEFAULT_WAIT_FOR_AGENT.wait_for(condition)
