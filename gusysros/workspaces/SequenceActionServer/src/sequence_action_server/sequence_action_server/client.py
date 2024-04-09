import rclpy.action
from rclpy.action import ActionClient
from rclpy.node import Node
from sys_actions.action import Sequence


class SequenceActionClient(Node):

    def __init__(self):
        super().__init__('sequence_action_client')
        self._action_client = ActionClient(self, Sequence, 'sequence')

    def send_goal(self, text):
        goal_msg = Sequence.Goal()
        goal_msg.goal = text
        self._action_client.wait_for_server()
        return self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

    def feedback_callback(self, feedback_msg):
        feedback: Sequence.Feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.feedback}')


def main(args=None):
    rclpy.init(args=args)
    action_client = SequenceActionClient()
    future = action_client.send_goal("Miaw")
    future = action_client.send_goal("Mow")
    future.result()
    rclpy.spin(action_client)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
