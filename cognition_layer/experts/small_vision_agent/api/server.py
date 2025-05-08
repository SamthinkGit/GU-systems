from operator import attrgetter

from cognition_layer.api import FastAgentProtocol
from cognition_layer.experts.small_vision_agent.core.small_vision_agent import (
    SmallVisionAgent,
)
from ecm.mediator.Interpreter import Interpreter
from ecm.shared import default_kwargs
from ecm.tools.registry import ItemRegistry


def get_fast_ap_server(interpreter: Interpreter, **kwargs) -> FastAgentProtocol:

    mva = SmallVisionAgent(
        **default_kwargs(
            {"registry": ItemRegistry()},
            kwargs,
        )
    )
    return FastAgentProtocol(
        name="SmallVisionAgent Server",
        iterator=lambda input: mva.invoke(input),
        step_name_getter=attrgetter("name"),
        content_getter=attrgetter("content"),
        is_last_getter=attrgetter("is_last"),
    )
