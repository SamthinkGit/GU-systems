import pytest  # noqa
import rclpy

from gusysalb.rosa import ROSA
from gusysros.tools.feedback import ExecutionStatus
from gusysros.tools.feedback import Feedback
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
            rclpy.shutdown()

        print(f"Feedback received: {feedback._exec_status}")

    def test_execute_mock(self):
        self.rosa.new_task(
            task_id=TestRosa.mock.task_id, feedback_callback=TestRosa.feedback_callback
        )
        pkg = TestRosa.mock.get_package()
        self.rosa.execute(pkg)
