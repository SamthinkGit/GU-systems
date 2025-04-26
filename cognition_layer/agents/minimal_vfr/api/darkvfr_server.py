from operator import attrgetter

from cognition_layer.agents.minimal_vfr.variations.darkvfr import DarkVFR
from cognition_layer.api import FastAgentProtocol
from ecm.mediator.Interpreter import Interpreter
from ecm.shared import default_kwargs
from ecm.tools.registry import ItemRegistry


def get_fast_ap_server(interpreter: Interpreter, **kwargs) -> FastAgentProtocol:

    vfr = DarkVFR(
        interpreter=interpreter,
        **default_kwargs(
            {"registry": ItemRegistry(), "memory_capacity": 10},
            kwargs,
        )
    )
    return FastAgentProtocol(
        name="DarkVFR Server",
        iterator=lambda input: vfr.complete_task(input),
        step_name_getter=attrgetter("name"),
        content_getter=attrgetter("content"),
        is_last_getter=attrgetter("is_last"),
    )
