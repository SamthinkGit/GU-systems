import time
from sequence_action_server.feedback_response import ResponseListener
from sequence_action_server.feedback_response import ResponsePublisher
from threading import Thread

import rclpy


class TestFeedback:

    def setup(self):
        rclpy.init()
        self.publisher_node = ResponsePublisher()
        self.listener_node = ResponseListener()

        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.publisher_node)
        self.executor.add_node(self.listener_node)
        self.executor_thread = Thread(target=self.executor.spin)
        self.executor_thread.start()

    def teardown(self):
        self.executor.shutdown()
        self.executor_thread.join()
        rclpy.shutdown()

    def waiting_callback(self):
        message = self.listener_node.wait_for_message("1234")
        self.received = message

    def test_wait_for_message(self):

        waiter = Thread(target=self.waiting_callback)
        waiter.start()
        time.sleep(1)
        response_code = "1234"
        self.publisher_node.publish_response(response_code + "Response")
        waiter.join()
        assert self.received == "Response"
