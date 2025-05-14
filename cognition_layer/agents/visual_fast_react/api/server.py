from operator import attrgetter

from cognition_layer.agents.visual_fast_react.agents.visual_fast_react import (
    VisualFastReact,
)
from cognition_layer.api import FastAgentProtocol
from ecm.mediator.Interpreter import Interpreter
from ecm.tools.registry import ItemRegistry


def get_fast_ap_server(
    interpreter: Interpreter,
    registry: ItemRegistry = ItemRegistry(),
    memory_capacity: int = 10,
    **kwargs
) -> FastAgentProtocol:

    vfr = VisualFastReact(
        interpreter=interpreter,
        memory_capacity=memory_capacity,
        registry=registry,
    )
    return FastAgentProtocol(
        name="VisualFastReact Server",
        iterator=lambda input: vfr.iter(input),
        step_name_getter=attrgetter("name"),
        content_getter=attrgetter("content"),
        is_last_getter=attrgetter("is_last"),
    )
