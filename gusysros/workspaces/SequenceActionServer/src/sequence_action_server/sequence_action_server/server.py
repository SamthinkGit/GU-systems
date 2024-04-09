import time

import rclpy.action
from rclpy.action import ActionServer
from rclpy.node import Node
from sys_actions.action import Sequence


class SequenceActionServer(Node):

    def __init__(self):
        super().__init__('sequence_action_server')
        self._action_server = ActionServer(
            self,
            Sequence,
            'sequence',
            self.execute_callback)

    def execute_callback(self, goal_handle: rclpy.action.server.ServerGoalHandle):
        feedback_msg = Sequence.Feedback()
        feedback_msg.feedback = "Im a feedback!"
        goal_handle.publish_feedback(feedback_msg)
        print("Started")
        print("Sleeping...")
        time.sleep(3)
        rclpy.spin_once(self, timeout_sec=0.1)
        print("Hi!")

        goal_handle.succeed()
        result = Sequence.Result()
        result.result = feedback_msg.feedback

        return result


def main(args=None):
    rclpy.init(args=args)
    sequence_action_server = SequenceActionServer()
    rclpy.spin(sequence_action_server)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
