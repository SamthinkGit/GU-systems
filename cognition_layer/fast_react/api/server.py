from cognition_layer.api import ServerAPI
from cognition_layer.fast_react.agents.fast_react import FastReact
from cognition_layer.tools.server_template import ServerFromIterator
from ecm.mediator.Interpreter import Interpreter
from ecm.shared import default_kwargs
from ecm.tools.registry import ItemRegistry
from operator import attrgetter


def get_server(interpreter_class: Interpreter, **kwargs) -> ServerAPI:
    react = FastReact(
        interpreter=interpreter_class(),
        **default_kwargs({"registry": ItemRegistry(), "memory_capacity": 10}, kwargs)
    )

    return ServerFromIterator(
        name="FastReact Server",
        iterator=lambda input: react.complete_task(input),
        async_iterator=False,
        step_name_getter=attrgetter("name"),
        content_getter=attrgetter("content"),
        is_last_getter=attrgetter("is_last"),
    )
