import atexit
import importlib
import logging
import os
import threading
from sequence_action_server.client import SequenceActionClient
from sequence_action_server.feedback import FeedbackPublisher
from sequence_action_server.server import SequenceActionServer

import rclpy
from rclpy.executors import MultiThreadedExecutor

from gusysalb.nodes import NodeRegistry
from gusyscore.core import get_root_path


class ALB:

    root_path = get_root_path()
    mocks = ["/gusyscore/gateway/mocks"]
    types = ["/gusysros/types"]
    nodes = {
        'sequence_server': SequenceActionServer,
        'sequence_client': SequenceActionClient,
        'feedback_client': FeedbackPublisher
    }

    _instance = None
    _built = False
    _clean = False

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self) -> None:
        pass

    def build_all(self):
        if not self._built:
            self._built = True

            self.load_mocks()
            self.load_types()
            self.load_nodes()

    def load_mocks(self):
        self.import_path(self.mocks)

    def load_types(self):
        self.import_path(self.types)

    def load_nodes(self):
        rclpy.init()
        NodeRegistry.inited_nodes = {name: node() for name, node in self.nodes.items()}

        self.executor = MultiThreadedExecutor()
        for node in NodeRegistry.inited_nodes.values():
            self.executor.add_node(node)

        thread = threading.Thread(target=self.executor.spin)
        logging.debug("Launching Nodes")
        thread.start()

        atexit.register(self.cleanup)

    def cleanup(self):
        if not self._clean:
            self._clean = True
            logging.debug("Closing Nodes")
            if self.executor:
                self.executor.shutdown()

            for node in NodeRegistry.inited_nodes.values():
                node.destroy_node()

    def import_path(self, directories: list):
        for dir in directories:
            path = str(self.root_path) + dir

            for filename in os.listdir(path):
                if filename.endswith(".py") and filename != "__init__.py":
                    module = dir[1:].replace("/", ".") + "." + filename[:-3]
                    logging.debug(f"Loading module {filename}")
                    importlib.import_module(module)


if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG, format="[%(levelname)s] %(message)s")
    alb = ALB()
    alb.build_all()
    print("Building Completed")
    input("Press ENTER to stop execution")
    alb.cleanup()
