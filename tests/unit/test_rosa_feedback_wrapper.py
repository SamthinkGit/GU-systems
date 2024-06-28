import pytest

from ecm.mediator.rosa_interpreter import ExecutionStatus
from ecm.mediator.rosa_interpreter import RosaFeedbackWrapper as Feedback
from ecm.tools.registry import ItemRegistry
from execution_layer.rosa.gateway.mocks.debug import hello_world
from execution_layer.rosa.interfaces.rosa import ROSA
from execution_layer.rosa.ros2.tools.packages import ActionPackage
from execution_layer.rosa.ros2.tools.packages import SequencePackage
from execution_layer.rosa.ros2.tools.packages import SequencePriority
from execution_layer.rosa.ros2.types.basic import ControlledSequence


def test_rosa_feedback_wrapper(capsys: pytest.CaptureFixture):
    rosa: ROSA = ROSA()
    action = ActionPackage(ItemRegistry.get_id(hello_world))
    seq = SequencePackage(
        task_id="hello",
        type=ControlledSequence.get_type(),
        priority=SequencePriority.NORMAL,
        actions=[action] * 3,
    )

    def feedback_callback(message):
        feedback: Feedback = Feedback.parse(message)
        if feedback._exec_status == ExecutionStatus.REQUEST_TO_CONTINUE:
            feedback.response(None, ExecutionStatus.CONTINUE)

    rosa.new_task("hello", feedback_callback=feedback_callback)
    rosa.execute(seq)
    rosa.wait_for("hello", ExecutionStatus.FINISH)

    capture = capsys.readouterr()
    assert "Hello World!\n"*3 in capture.out
