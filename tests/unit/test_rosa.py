import time

import pytest  # noqa

from execution_layer.rosa.gateway.mocks.debug import sleep_and_print
from execution_layer.rosa.interfaces.rosa import ROSA
from execution_layer.rosa.ros2.tools.feedback import ExecutionStatus
from execution_layer.rosa.ros2.tools.feedback import Feedback
from execution_layer.rosa.ros2.tools.packages import ActionPackage
from execution_layer.rosa.ros2.tools.packages import SequencePackage
from execution_layer.rosa.ros2.tools.packages import SequencePriority
from execution_layer.rosa.ros2.tools.registry import ItemRegistry
from execution_layer.rosa.ros2.types.basic import SimpleSequence
from tests.mocks.packages import PackageMock


class TestRosa:

    rosa: ROSA
    mock = None

    def setup(self):
        self.rosa = ROSA()
        TestRosa.mock = PackageMock()

    def test_wait_for(self, capsys: pytest.CaptureFixture):
        wait_for_text = ActionPackage(
            action_id=ItemRegistry.get_id(sleep_and_print), text="wait_for_text"
        )
        end_text = ActionPackage(
            action_id=ItemRegistry.get_id(sleep_and_print), text="end_text"
        )
        seq = SequencePackage(
            task_id="test_wait_for",
            type=SimpleSequence.get_type(),
            priority=SequencePriority.NORMAL,
            actions=[wait_for_text, end_text],
        )
        self.rosa.new_task("test_wait_for", feedback_callback=ROSA.muted_callback)
        self.rosa.execute(seq)
        self.rosa.wait_for("test_wait_for", ExecutionStatus.FINISH)

        capture = capsys.readouterr()
        assert "end_text" in capture.out

    @staticmethod
    def feedback_callback(feedback: Feedback):
        if feedback._exec_status == ExecutionStatus.FINISH:
            assert TestRosa.mock.list_properly_modified()

    def test_execute_mock(self):
        self.rosa.new_task(
            task_id=TestRosa.mock.task_id, feedback_callback=TestRosa.feedback_callback
        )
        pkg = TestRosa.mock.get_package()
        pkg.task_id = "test_execute_mock"
        self.rosa.execute(pkg)
        self.rosa.wait_for(pkg.task_id, code=ExecutionStatus.FINISH)

    def test_soft_stop(self, capsys: pytest.CaptureFixture):
        task_id = "test_soft_stop"
        text_wait = ActionPackage(
            action_id=ItemRegistry.get_id(sleep_and_print), text="WAIT"
        )
        text_finish = ActionPackage(
            action_id=ItemRegistry.get_id(sleep_and_print), text="FINISH"
        )
        seq = SequencePackage(
            task_id=task_id,
            type=SimpleSequence.get_type(),
            priority=SequencePriority.NORMAL,
            actions=[text_wait] * 10 + [text_finish],
        )
        self.rosa.new_task(task_id=task_id, feedback_callback=ROSA.muted_callback)
        self.rosa.execute(seq)
        self.rosa.wait_for(task_id=task_id, code=ExecutionStatus.STEP)
        self.rosa.soft_stop(task_id=task_id)
        self.rosa.wait_for(task_id=task_id, code=ExecutionStatus.FINISH)

        capture = capsys.readouterr()
        assert "FINISH" not in capture.out

    # ---- This test can be checked manually, although it is dangerous and should be cleaned
    # if python processes keep alive
    def hard_stop(self, capsys: pytest.CaptureFixture):
        task_id = "test_hard_stop"
        text_wait = ActionPackage(
            action_id=ItemRegistry.get_id(sleep_and_print), text="WAIT_HARD"
        )
        text_finish = ActionPackage(
            action_id=ItemRegistry.get_id(sleep_and_print), text="FINISH_HARD"
        )
        seq = SequencePackage(
            task_id=task_id,
            type=SimpleSequence.get_type(),
            priority=SequencePriority.NORMAL,
            actions=[text_wait] * 10 + [text_finish],
        )

        self.rosa.new_task(task_id=task_id)
        self.rosa.execute(seq)
        self.rosa.wait_for(task_id=task_id, code=ExecutionStatus.STEP)
        self.rosa._hard_stop(task_id=task_id)
        time.sleep(1)

        capture = capsys.readouterr()
        assert "FINISH_HARD" not in capture.out
