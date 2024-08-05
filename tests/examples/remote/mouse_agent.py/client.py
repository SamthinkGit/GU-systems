import action_space.experimental.mouse.actions # noqa
import action_space.experimental.screenshot.actions # noqa
from ecm.remote.client import EcmClient

if __name__ == "__main__":
    client = EcmClient()
    client.listen()
