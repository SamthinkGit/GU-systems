"""
IODA Client / Request Listener
====================================

The responsability of this module then is to listen to new packages at the
/request topic and send them to the execution server only when the task is
READY, this is, according to the TSP protocol

For this, this module uses ROS 2 action clients into task management systems,
facilitating the processing of sequential actions based on tasks received.
"""
import traceback
from sequence_action_server.observer import Publisher
from sequence_action_server.observer import Subscriber

from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from std_msgs.msg import String
from sys_actions.action import Sequence

from ecm.shared import get_logger
from execution_layer.rosa.constants import REQUEST_TOPIC
from execution_layer.rosa.ros2.tools.packages import SequencePackage
from execution_layer.rosa.ros2.tools.packages import TaskRegistry
from execution_layer.rosa.ros2.tools.packages import TaskStatus


class SequenceActionClient(Node):
    """
    ROS 2 Node acting as an action client for managing sequences of tasks.
    """

    _logger = get_logger("ActionClient")

    def __init__(self):
        """
        Initializes the SequenceActionClient node, setting up subscribers and action clients.
        """

        super().__init__("sequence_action_client")

        self.registry = TaskRegistry()
        self.subscriber = Subscriber(target=self.goal_completed_callback)
        Publisher().add_subscriber(self.subscriber)

        self._action_client = ActionClient(self, Sequence, "sequence")
        self.subscription = self.create_subscription(
            String, REQUEST_TOPIC, self.request_callback, qos_profile_system_default
        )

    def send_goal(self, text):
        """
        Sends the package received to the ROS 2 action server.
        :param text: Serialized JSON text of the sequence package.
        """
        goal_msg = Sequence.Goal()
        goal_msg.goal = text
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

    def request_callback(self, msg):
        """
        Handles incoming requests, processes the sequence package, and initiates actions based on TSP.
        :param msg: Message containing the serialized JSON of a sequence package.
        """
        try:
            sequence = SequencePackage.from_json(msg.data)
            task_id = sequence.task_id
        except Exception:
            trback = traceback.format_exc()
            SequenceActionClient._logger.warn(
                f"Invalid package received in Sequence Action client. {trback}"
            )
            return
        SequenceActionClient._logger.debug(f"Sequence <{task_id}> received")

        # Add the sequence to the registry
        self.registry.update(sequence)

        # Look if the task is currently working
        if self.registry.tasks[task_id].status == TaskStatus.READY:
            SequenceActionClient._logger.debug(f"Sending <{task_id}> to the server")

            # Execute the next task according to STP
            self.registry.tasks[task_id].status = TaskStatus.RUNNING
            sequence = self.registry.get(task_id)
            if sequence is None:
                SequenceActionClient._logger.warn(
                    f"It looks like <{task_id} >has been removed from the TaskRegistry, Skip."
                )
                self.registry.tasks[task_id].status = TaskStatus.READY
            else:
                self.send_goal(sequence.to_json())

    def goal_completed_callback(self, task_id):
        """
        Callback for when a sequence has completed, either moving to the next sequence in the same task
        or setting the task status to READY.
        :param task_id: The identifier of the task that has completed.
        """
        SequenceActionClient._logger.debug("Task Completed")
        next_sequence = self.registry.get(task_id)

        if next_sequence is None:
            SequenceActionClient._logger.debug(
                f"All Sequences for {task_id} have been completed"
            )
            self.registry.tasks[task_id].status = TaskStatus.READY
        else:
            SequenceActionClient._logger.debug("Starting Next Sequence")
            self.send_goal(next_sequence.to_json())
