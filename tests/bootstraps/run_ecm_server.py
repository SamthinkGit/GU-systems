import action_space.keyboard.virtual_keyboard # noqa
import ecm.exelent.parser as parser
from ecm.remote.server import EcmServer
from ecm.shared import get_root_path
from ecm.tools.registry import ItemRegistry
from execution_layer.rosa.interpreter.rosa_interpreter import RosaInterpreter

if __name__ == "__main__":

    # CAUTION: This script is used fot the test: test_ecm_server.py

    ItemRegistry.transfer_execution_to_client()
    server = EcmServer()

    rosa = RosaInterpreter()
    path = get_root_path() / "tests" / "resources" / "write.xlnt"
    task = parser.parse(path)
    rosa.run(task, callback="mute")
    rosa.kill()
