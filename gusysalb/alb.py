"""
ALB (Application Layer Builder)
=================================

This module manages the setup and tear-down of an application's
various components within a ROS 2 environment. It dynamically
loads and initializes specific node types and additional Python
modules, integrating them into a ROS 2 execution framework with
multi-threaded support.

Note that the ALB ensures that all nodes and types remain in the
same process space, enabling easier object passing (using for
example, TaskRegistry)
"""
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
    """
    [Singleton] Class designed to initialize and manage the lifecycle of a ROS 2 application's nodes and modules.
    The ALB is responsible for loading mocks, types, and ROS 2 nodes dynamically
    at runtime, providing a flexible execution environment.
    """

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
        """
        Ensures that only one instance of the ALB exists (singleton pattern).
        """
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self) -> None:
        pass

    def build_all(self):
        """
        Constructs and initializes all components necessary for the application.
        """
        if not self._built:
            self._built = True

            self.load_mocks()
            self.load_types()
            self.load_nodes()

    def load_mocks(self):
        """
        Dynamically imports Python modules for mocks (also registering them).
        """
        self.import_path(self.mocks)

    def load_types(self):
        """
        Dynamically imports Python modules for types (also registering them).
        """
        self.import_path(self.types)

    def load_nodes(self):
        """
        Initializes and registers all ROS 2 nodes and starts the multi-threaded
        executor to handle these nodes.
        """
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
        """
        Cleans up all resources, ensuring a proper shutdown of
        the multi-threaded executor and destruction of all nodes.
        """
        if not self._clean:
            self._clean = True
            logging.debug("Closing Nodes")
            if self.executor:
                self.executor.shutdown()

            for node in NodeRegistry.inited_nodes.values():
                node.destroy_node()

    def import_path(self, directories: list):
        """
        Helper method to import modules from the specified directories
        within the project's root path.
        """
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
