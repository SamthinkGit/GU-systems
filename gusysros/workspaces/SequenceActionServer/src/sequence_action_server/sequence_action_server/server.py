import traceback

import rclpy.action
from rclpy.action import ActionServer
from rclpy.node import Node
from sys_actions.action import Sequence

from gusysros.tools.packages import SequencePackage
from gusysros.types.basic import SequenceType


class SequenceActionServer(Node):

    def __init__(self):
        super().__init__('sequence_action_server')
        self._action_server = ActionServer(
            self,
            Sequence,
            'sequence',
            self.execute_callback)

    def execute_callback(self, goal_handle: rclpy.action.server.ServerGoalHandle):
        # feedback_msg = Sequence.Feedback()
        # feedback_msg.feedback = "Function Executed"
        # goal_handle.publish_feedback(feedback_msg)
        try:
            sequence_pkg = SequencePackage.from_json(goal_handle.request.goal)
        except Exception:
            trback = traceback.format_exc()
            self.get_logger().warn(f"Invalid package received in Sequence Action client. {trback}")
            return

        seq_type = SequenceType.from_pkg(sequence_pkg)
        seq_type.run()

        goal_handle.succeed()
        result = Sequence.Result()
        result.result = "SUCCESS"

        return result


def main(args=None):
    rclpy.init(args=args)
    sequence_action_server = SequenceActionServer()
    rclpy.spin(sequence_action_server)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
