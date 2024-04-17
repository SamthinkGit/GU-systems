import json
from enum import Enum
from sequence_action_server.feedback import FeedbackPublisher
from typing import Any

from gusysalb.nodes import NodeRegistry
from gusysros.tools.registry import ItemEncoder
from gusysros.tools.registry import ThreadRegistry


class ExecutionStatus(Enum):
    RUNNING = 0
    STEP = 1
    SWITCH = 2


class Feedback:

    def __init__(self) -> None:
        self.publisher: FeedbackPublisher = NodeRegistry.inited_nodes["feedback_client"]

    def publish(self, object: Any, _status: ExecutionStatus = ExecutionStatus.RUNNING):
        message = {
            'task_id': ThreadRegistry.get_task_id(),
            'object': ItemEncoder.autoencode(object),
            '_exec_status': ItemEncoder.autoencode(_status)
        }
        json_pkg = json.dumps(message, indent=4)
        self.publisher.publish_feedback(json_pkg)
