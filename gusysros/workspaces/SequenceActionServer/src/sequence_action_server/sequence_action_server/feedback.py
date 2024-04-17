import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from gusyscore.constants import FEEDBACK_TOPIC


class FeedbackPublisher(Node):
    def __init__(self):
        super().__init__("feedback_publisher")
        self.publisher_ = self.create_publisher(String, FEEDBACK_TOPIC, 100)

    def publish_feedback(self, message: str):
        msg = String()
        msg.data = message
        self.publisher_.publish(msg)


def main():
    rclpy.init()
    feedback_publisher = FeedbackPublisher()
    try:
        while True:
            message = input("Enter a feedback message to publish: ")
            feedback_publisher.publish_feedback(message)
    except KeyboardInterrupt:
        pass
    feedback_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
