from operator import attrgetter

from cognition_layer.api import FastAgentProtocol
from cognition_layer.visual_fast_react.agents.visual_fast_react import VisualFastReact
from ecm.mediator.Interpreter import Interpreter
from ecm.shared import default_kwargs
from ecm.tools.registry import ItemRegistry


def get_fast_ap_server(interpreter: Interpreter, **kwargs) -> FastAgentProtocol:

    vfr = VisualFastReact(
        interpreter=interpreter,
        **default_kwargs(
            {"registry": ItemRegistry(), "memory_capacity": 10},
            kwargs,
        )
    )
    return FastAgentProtocol(
        name="VisualFastReact Server",
        iterator=lambda input: vfr.iter(input),
        step_name_getter=attrgetter("name"),
        content_getter=attrgetter("content"),
        is_last_getter=attrgetter("is_last"),
    )
