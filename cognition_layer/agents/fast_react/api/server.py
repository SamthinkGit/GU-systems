from operator import attrgetter

from cognition_layer.agents.fast_react.agents.fast_react import FastReact
from cognition_layer.api import FastAgentProtocol
from cognition_layer.api import ServerAPI
from cognition_layer.tools.server_template import ServerFromIterator
from ecm.mediator.Interpreter import Interpreter
from ecm.tools.registry import ItemRegistry


def get_server(
    interpreter_class: Interpreter,
    registry: ItemRegistry = ItemRegistry(),
    memory_capacity: int = 10,
    **kwargs,
) -> ServerAPI:
    react = FastReact(
        interpreter=interpreter_class(),
        registry=registry,
        memory_capacity=memory_capacity,
    )

    return ServerFromIterator(
        name="FastReact Server",
        iterator=lambda input: react.complete_task(input),
        async_iterator=False,
        step_name_getter=attrgetter("name"),
        content_getter=attrgetter("content"),
        is_last_getter=attrgetter("is_last"),
    )


def get_fast_ap_server(
    interpreter: Interpreter,
    registry: ItemRegistry = ItemRegistry(),
    memory_capacity: int = 10,
    **kwargs,
) -> FastAgentProtocol:

    react = FastReact(
        interpreter=interpreter,
        registry=registry,
        memory_capacity=memory_capacity,
        ocr_mode=True,
    )
    return FastAgentProtocol(
        name="FastReact Server",
        iterator=lambda input: react.complete_task(input),
        step_name_getter=attrgetter("name"),
        content_getter=attrgetter("content"),
        is_last_getter=attrgetter("is_last"),
    )
