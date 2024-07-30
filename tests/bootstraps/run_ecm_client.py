import action_space.keyboard.virtual_keyboard # noqa
from ecm.remote.client import EcmClient
from ecm.tools.registry import ItemRegistry

if __name__ == "__main__":
    # CAUTION: This script is used fot the test: test_ecm_server.py

    ItemRegistry.invalidate_all_functions()
    client = EcmClient()
    client.listen()
