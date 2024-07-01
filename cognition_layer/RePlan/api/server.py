from operator import attrgetter

from cognition_layer.api import ServerAPI
from cognition_layer.RePlan.agents.replan import RePlan
from cognition_layer.tools.server_template import ServerFromIterator
from ecm.mediator.Interpreter import Interpreter
from ecm.tools.async_interpreter import AsyncInterpreter


def get_server(
    verbose, interpreter: Interpreter, max_planex_steps: int = 11
) -> ServerAPI:
    async_interpreter = AsyncInterpreter()
    async_interpreter.build(
        interpreter=interpreter, feedback_class=interpreter.feedback_message_class
    )
    replan = RePlan(
        async_interpreter=async_interpreter,
        verbose=verbose,
        max_planex_steps=max_planex_steps,
    )
    return ServerFromIterator(
        name="RePlan",
        iterator=lambda input: replan.iter(input),
        async_iterator=True,
        step_name_getter=attrgetter("name"),
        content_getter=attrgetter("content"),
        is_last_getter=attrgetter("is_last"),
    )
