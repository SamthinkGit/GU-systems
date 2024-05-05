import pytest

from execution_layer.rosa.ros2.tools.registry import ThreadRegistry

father_id = None


def mock_function(num):
    print(f"Num received is {num}")
    print(f"Executed function with task_id: {ThreadRegistry.get_task_id()}")
    assert father_id != ThreadRegistry.get_task_id()


def test_thread_registry(capsys: pytest.CaptureFixture):

    task_id = "my_task"
    tr = ThreadRegistry()
    father_id = ThreadRegistry.get_task_id()

    tr.watch(task_id=task_id, target=mock_function, num=5)
    print(f"Father task_id is {father_id}")
    print("Waiting for task...")

    tr.wait(task_id)
    capture = capsys.readouterr()
    assert "Executed function" in capture.out
