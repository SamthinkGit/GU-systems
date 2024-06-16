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
import os
import threading
from sequence_action_server.client import SequenceActionClient
from sequence_action_server.feedback import FeedbackListener
from sequence_action_server.feedback import FeedbackPublisher
from sequence_action_server.registry_logger import RegistryLogger
from sequence_action_server.sequence_publisher import SequencePublisher
from sequence_action_server.server import SequenceActionServer
from typing import Callable

import rclpy
from rclpy.executors import MultiThreadedExecutor

from ecm.shared import get_logger
from ecm.shared import get_root_path
from execution_layer.rosa.interfaces.nodes import NodeRegistry
from execution_layer.rosa.shared import ignore_invalid_warnings as ignore_warns


class ALB:
    """
    [Singleton] Class designed to initialize and manage the lifecycle of a ROS 2 application's nodes and modules.
    The ALB is responsible for loading mocks, types, and ROS 2 nodes dynamically
    at runtime, providing a flexible execution environment.
    """

    root_path = get_root_path()
    mocks = ["/execution_layer/rosa/gateway/mocks"]
    types = ["/execution_layer/rosa/ros2/types"]
    nodes = {
        "sequence_server": SequenceActionServer,
        "sequence_client": SequenceActionClient,
        "feedback_publisher": FeedbackPublisher,
        "sequence_publisher": SequencePublisher,
        "registry_logger": RegistryLogger,
    }

    _logger = get_logger("ALB")
    _instance = None
    _built = False
    _clean = False
    _feedback_listener = None

    def __new__(cls):
        """
        Ensures that only one instance of the ALB exists (singleton pattern).
        """
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self, ignore_invalid_warnings: bool = True) -> None:
        if ignore_invalid_warnings:
            ignore_warns()

    def build_all(self, feedback_listener: Callable = None):
        """
        Constructs and initializes all components necessary for the application.
        """
        if not self._built:
            self._built = True

            if feedback_listener is not None:
                self._feedback_listener = feedback_listener
            self.load_mocks()
            self.load_types()
            self.load_nodes()
        else:
            self._logger.warn(
                "Trying to build ALB after it has already been built. Skipping"
            )

    def load_mocks(self):
        """
        Dynamically imports Python modules for mocks (also registering them).
        """
        self._logger.debug("Building Mocks...")
        self.import_path(self.mocks)

    def load_types(self):
        """
        Dynamically imports Python modules for types (also registering them).
        """
        self._logger.debug("Building Types...")
        self.import_path(self.types)

    def load_nodes(self):
        """
        Initializes and registers all ROS 2 nodes and starts the multi-threaded
        executor to handle these nodes.
        """
        self._logger.debug("Launching Nodes...")
        rclpy.init()
        NodeRegistry.inited_nodes = {name: node() for name, node in self.nodes.items()}

        self.executor = MultiThreadedExecutor()
        for node in NodeRegistry.inited_nodes.values():
            self.executor.add_node(node)

        if self._feedback_listener is not None:
            feedback_node = FeedbackListener(self._feedback_listener)
            self.executor.add_node(feedback_node)
            NodeRegistry.inited_nodes["feedback_listener"] = feedback_node

        thread = threading.Thread(target=self.executor.spin)
        thread.start()
        self._logger.debug("All nodes have been launched in a new thread")

        atexit.register(self.cleanup)

    def cleanup(self):
        """
        Cleans up all resources, ensuring a proper shutdown of
        the multi-threaded executor and destruction of all nodes.
        """
        if not self._clean:
            self._clean = True
            self._logger.debug("Closing Nodes")
            if self.executor:
                self.executor.shutdown()

            for node in NodeRegistry.inited_nodes.values():
                node.destroy_node()

        self._logger.debug("Cleanup Completed")

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
                    self._logger.debug(f"Loading module {filename}")
                    importlib.import_module(module)


if __name__ == "__main__":
    alb = ALB()
    alb.build_all()
    print("Building Completed")
    input("Press ENTER to stop execution")
    alb.cleanup()
