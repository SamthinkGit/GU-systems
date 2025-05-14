from operator import attrgetter

from cognition_layer.api import FastAgentProtocol
from cognition_layer.experts.small_vision_agent.core.small_vision_agent import (
    SmallVisionAgent,
)
from ecm.tools.registry import ItemRegistry


def get_fast_ap_server(
    registry: ItemRegistry = ItemRegistry(), **kwargs
) -> FastAgentProtocol:

    mva = SmallVisionAgent(registry=registry)
    return FastAgentProtocol(
        name="SmallVisionAgent Server",
        iterator=lambda input: mva.invoke(input),
        step_name_getter=attrgetter("name"),
        content_getter=attrgetter("content"),
        is_last_getter=attrgetter("is_last"),
    )
