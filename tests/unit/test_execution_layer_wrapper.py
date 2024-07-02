import time

from ecm.mediator.execution_layer_wrapper import ExecutionLayerWrapper
from ecm.mediator.feedback import ExecutionStatus
from ecm.mediator.feedback import Feedback
from ecm.shared import get_root_path
from ecm.tools.registry import ItemRegistry
from execution_layer.rosa.interpreter.rosa_interpreter import RosaFeedbackWrapper
from execution_layer.rosa.interpreter.rosa_interpreter import RosaInterpreter


@ItemRegistry.register_function
def click():
    """Click"""
    pass


def test_execution_layer_wrapper():

    path = get_root_path() / "tests" / "resources" / "controlled_plan.xlnt"
    with open(file=path, encoding="utf-8") as fd:
        exelent_code = fd.read()

    ExecutionLayerWrapper.build(interpreter_class=RosaInterpreter, feedback_class=RosaFeedbackWrapper)
    ExecutionLayerWrapper.set_exelent_code(exelent_code)
    ExecutionLayerWrapper.start_execution()

    status = None
    requests_accepted = 0
    while status != ExecutionStatus.FINISH:
        if len(ExecutionLayerWrapper.history) == 0:
            time.sleep(0.2)
            continue

        with ExecutionLayerWrapper._lock:
            feedback: Feedback = ExecutionLayerWrapper.history[-1]
            status = feedback._exec_status
            if status == ExecutionStatus.REQUEST_TO_CONTINUE:
                requests_accepted += 1
                feedback.response(None, ExecutionStatus.CONTINUE)
            ExecutionLayerWrapper.history.pop()

    assert requests_accepted == 4
