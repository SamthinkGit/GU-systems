import action_space.keyboard.actions  # noqa
from ecm.remote.client import EcmClient
from ecm.shared import get_logger

logger = get_logger("Main")

if __name__ == "__main__":
    client = EcmClient()
    client.listen()
