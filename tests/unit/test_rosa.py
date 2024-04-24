import time

import pytest  # noqa

from gusysalb.rosa import ROSA
from gusyscore.gateway.mocks.debug import sleep_and_print
from gusysros.tools.feedback import ExecutionStatus
from gusysros.tools.feedback import Feedback
from gusysros.tools.packages import ActionPackage
from gusysros.tools.packages import SequencePackage
from gusysros.tools.packages import SequencePriority
from gusysros.tools.registry import ItemRegistry
from gusysros.types.basic import SimpleSequence
from tests.mocks.packages import PackageMock


class TestRosa:

    rosa: ROSA
    mock = None

    def setup(self):
        self.rosa = ROSA()
        TestRosa.mock = PackageMock()

    @staticmethod
    def feedback_callback(feedback: Feedback):
        if feedback._exec_status == ExecutionStatus.SUCCESS:
            assert TestRosa.mock.list_properly_modified()

        print(f"Feedback received: {feedback._exec_status}")

    def test_execute_mock(self):
        self.rosa.new_task(
            task_id=TestRosa.mock.task_id, feedback_callback=TestRosa.feedback_callback
        )
        pkg = TestRosa.mock.get_package()
        self.rosa.execute(pkg)

    def test_soft_stop(self, capsys: pytest.CaptureFixture):
        text_wait = ActionPackage(
            action_id=ItemRegistry.get_id(sleep_and_print), text="WAIT"
        )
        text_finish = ActionPackage(
            action_id=ItemRegistry.get_id(sleep_and_print), text="FINISH"
        )
        seq = SequencePackage(
            task_id="default",
            type=SimpleSequence.get_type(),
            priority=SequencePriority.NORMAL,
            actions=[text_wait, text_wait, text_finish],
        )
        rosa = ROSA()

        rosa.new_task(task_id="test_soft_stop")
        rosa.execute(seq)
        time.sleep(2)
        rosa.soft_stop()
        time.sleep(2)

        capture = capsys.readouterr()
        assert "FINISH" not in capture.out
