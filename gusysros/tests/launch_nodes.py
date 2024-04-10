from sequence_action_server.client import SequenceActionClient
from sequence_action_server.server import SequenceActionServer

import rclpy
from rclpy.executors import MultiThreadedExecutor

from gusysros.tests.send_sequence import test_function # noqa
# Imports for adding some mock functions


def main(args=None):
    rclpy.init(args=args)
    server = SequenceActionServer()
    client = SequenceActionClient()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(server)
    executor.add_node(client)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        client.destroy_node()
        server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
