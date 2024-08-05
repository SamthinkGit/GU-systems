import sys

from action_space.experimental.mouse.agent import MouseAgent
from ecm.tools.registry import ItemRegistry

if __name__ == "__main__":
    ItemRegistry.transfer_execution_to_client()

    MouseAgent.find(sys.argv[1])
