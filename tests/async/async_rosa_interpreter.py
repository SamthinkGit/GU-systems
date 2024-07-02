import asyncio

from ecm.mediator.feedback import ExecutionStatus
from ecm.mediator.feedback import Feedback
from ecm.shared import get_root_path
from ecm.tools.async_interpreter import AsyncInterpreter
from ecm.tools.registry import ItemRegistry
from execution_layer.rosa.interpreter.rosa_interpreter import RosaFeedbackWrapper
from execution_layer.rosa.interpreter.rosa_interpreter import RosaInterpreter


@ItemRegistry.register_function
def click():
    """Click"""
    pass


async def async_interpreter():

    rosa = RosaInterpreter()
    path = get_root_path() / "tests" / "resources" / "controlled_plan.xlnt"
    with open(file=path, encoding="utf-8") as fd:
        exelent_code = fd.read()

    AsyncInterpreter.build(interpreter=rosa, feedback_class=RosaFeedbackWrapper)
    AsyncInterpreter.set_exelent_code(exelent_code)
    await AsyncInterpreter.start_execution()

    requests_accepted = 0
    status = None
    while status != ExecutionStatus.FINISH:
        feedback: Feedback = await AsyncInterpreter.get_feedback()
        status = feedback._exec_status
        if status == ExecutionStatus.REQUEST_TO_CONTINUE:
            requests_accepted += 1
            feedback.response(None, ExecutionStatus.CONTINUE)

    assert requests_accepted == 4


def test_async_interpreter():
    asyncio.run(async_interpreter())
