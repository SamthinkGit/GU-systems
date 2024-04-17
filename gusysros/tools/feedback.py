import json
from sequence_action_server.feedback import FeedbackPublisher
from typing import Any

from gusysalb.alb import ALB
from gusysros.tools.registry import ItemEncoder


class Feedback:

    def __init__(self, task_id) -> None:
        alb = ALB()
        self.task_id = task_id
        self.publisher: FeedbackPublisher = alb.inited_nodes["feedback_client"]

    def publish(self, object: Any):
        message = {
            'task_id': self.task_id,
            'object': ItemEncoder.autoencode(object)
        }
        json_pkg = json.dumps(message, indent=4)
        self.publisher.publish_feedback(json_pkg)
