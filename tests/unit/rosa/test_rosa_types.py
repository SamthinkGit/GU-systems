import time

import pytest

from ecm.tools.registry import ItemRegistry
from execution_layer.rosa.interfaces.rosa import ROSA
from execution_layer.rosa.ros2.tools.feedback import ExecutionStatus
from execution_layer.rosa.ros2.tools.feedback import Feedback
from execution_layer.rosa.ros2.tools.packages import ActionPackage
from execution_layer.rosa.ros2.tools.packages import SequencePackage
from execution_layer.rosa.ros2.tools.packages import SequencePriority
from execution_layer.rosa.ros2.types.basic import ControlledSequence


class TestRosaTypes:

    def setup(self):
        self.rosa = ROSA()

    def test_controlled_sequence(self, capsys: pytest.CaptureFixture):

        @ItemRegistry.register_function
        def fake_print(msg: str):
            print(msg)

        def my_controller(feedback: Feedback):
            #            print(f"[{feedback.task_id}, {feedback.object}, {feedback._exec_status}]")
            if feedback._exec_status != ExecutionStatus.REQUEST_TO_CONTINUE:
                return
            feedback.approve()
            time.sleep(0.5)
            print("Request Approved")

        task_id = "controlled_print"

        action_1 = ActionPackage(
            action_id=ItemRegistry.get_id(fake_print), msg="step_1"
        )
        action_2 = ActionPackage(
            action_id=ItemRegistry.get_id(fake_print), msg="step_2"
        )
        seq = SequencePackage(
            task_id=task_id,
            type=ControlledSequence.get_type(),
            priority=SequencePriority.NORMAL,
            actions=[action_1, action_2],
        )

        self.rosa.new_task(task_id, feedback_callback=my_controller)
        self.rosa.execute(seq)
        self.rosa.wait_for(task_id, ExecutionStatus.FINISH)

        capture = capsys.readouterr()
        assert "step_1\nRequest Approved\nstep_2" in capture.out
