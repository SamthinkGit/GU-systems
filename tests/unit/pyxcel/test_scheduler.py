import threading
import time

from ecm.exelent.parser import ParsedAction
from ecm.mediator.feedback import ExecutionStatus
from ecm.tools.item_registry_v2 import ItemRegistry
from execution_layer.pyxcel.core.scheduler import Feedback
from execution_layer.pyxcel.core.scheduler import Sequential

scheduler_target = False
scheduler_lock = threading.Lock()
scheduler_feedback_target = []


def test_sequential_action():
    global scheduler_target
    id = "sequential_test"

    @ItemRegistry.register(type="action")
    def sequential_test():
        global scheduler_target
        scheduler_target = True

    registry = ItemRegistry(id)
    registry.load_all()

    mock = ParsedAction(name=id, args=[], kwargs={})
    seq = Sequential(actions=[mock], registry=registry)
    seq.run()
    assert scheduler_target, "Sequence.run has not completed the task."

    scheduler_target = False
    seq.arun()
    seq.clean()
    assert scheduler_target, "Sequence.arun has not completed the task."
    scheduler_target = False

    ItemRegistry.flush()


def test_arun_vs_run():
    global scheduler_target
    id = "sequential_test_2"

    @ItemRegistry.register(type="action")
    def sequential_test_2():
        global scheduler_target
        scheduler_target = True
        time.sleep(1)

    registry = ItemRegistry(id)
    registry.load_all()

    mock = ParsedAction(name=id, args=[], kwargs={})
    seq = Sequential(actions=[mock], registry=registry)
    start = time.perf_counter()
    seq.run()
    end = time.perf_counter()
    assert scheduler_target, "Sequence.run has not completed the task."
    assert end - start > 1, "Sequence has not been a blocking call"

    scheduler_target = False
    start = time.perf_counter()
    seq.arun()
    end = time.perf_counter()
    seq.clean()
    assert scheduler_target, "Sequence.arun has not completed the task."
    assert end - start < 1, "Sequence has been a blocking call"
    scheduler_target = False

    ItemRegistry.flush()


def test_stopping():

    global scheduler_target
    global scheduler_lock
    id = "sequential_test_3"

    @ItemRegistry.register(type="action")
    def sequential_test_3_a():
        global scheduler_lock
        with scheduler_lock:
            pass

    @ItemRegistry.register(type="action")
    def sequential_test_3_b():
        global scheduler_target
        scheduler_target = True

    registry = ItemRegistry(id)
    registry.load_all()

    mock = ParsedAction(name="sequential_test_3_a", args=[], kwargs={})
    mock2 = ParsedAction(name="sequential_test_3_b", args=[], kwargs={})
    seq = Sequential(actions=[mock, mock2], registry=registry)
    with scheduler_lock:
        seq.arun()
        seq.stop()
    seq.clean()
    assert not scheduler_target, "Sequence has not been soft-stopped."

    ItemRegistry.flush()


def test_feedback():
    global scheduler_feedback_target
    id = "sequential_test_4"

    @ItemRegistry.register(type="action")
    def sequential_test_4():
        return "RESULT"

    def callback(feedback: Feedback):
        global scheduler_feedback_target
        if feedback._exec_status == ExecutionStatus.RESULT:
            scheduler_feedback_target.append(feedback.object)

        elif feedback._exec_status == ExecutionStatus.STEP:
            scheduler_feedback_target.append("STEP")

        elif feedback._exec_status == ExecutionStatus.SUCCESS:
            scheduler_feedback_target.append("SUCCESS")

        elif feedback._exec_status == ExecutionStatus.FINISH:
            scheduler_feedback_target.append("FINISH")

    registry = ItemRegistry(id)
    registry.load_all()

    mock = ParsedAction(name="sequential_test_4", args=[], kwargs={})
    seq = Sequential(actions=[mock], registry=registry, feedback_callback=callback)
    seq.run()

    assert "STEP" in scheduler_feedback_target
    assert "SUCCESS" in scheduler_feedback_target
    assert "FINISH" in scheduler_feedback_target
    assert "RESULT" in scheduler_feedback_target

    ItemRegistry.flush()
