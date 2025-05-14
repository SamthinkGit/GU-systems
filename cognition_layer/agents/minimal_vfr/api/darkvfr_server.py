from operator import attrgetter

from cognition_layer.agents.minimal_vfr.variations.darkvfr import DarkVFR
from cognition_layer.api import FastAgentProtocol
from ecm.mediator.Interpreter import Interpreter
from ecm.tools.registry import ItemRegistry


def get_fast_ap_server(
    interpreter: Interpreter,
    registry: ItemRegistry = ItemRegistry(),
    memory_capacity: int = 10,
    **kwargs
) -> FastAgentProtocol:

    vfr = DarkVFR(
        interpreter=interpreter,
        registry=registry,
        memory_capacity=memory_capacity,
    )
    return FastAgentProtocol(
        name="DarkVFR Server",
        iterator=lambda input: vfr.complete_task(input),
        step_name_getter=attrgetter("name"),
        content_getter=attrgetter("content"),
        is_last_getter=attrgetter("is_last"),
    )
