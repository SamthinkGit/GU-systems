from operator import attrgetter

from cognition_layer.api import FastAgentProtocol
from cognition_layer.experts.element_finder.agent import (
    ElementFinder,
)
from ecm.tools.registry import ItemRegistry


def get_fast_ap_server(
    registry: ItemRegistry = ItemRegistry(), *args, **kwargs
) -> FastAgentProtocol:

    finder = ElementFinder(registry=registry, *args, **kwargs)
    return FastAgentProtocol(
        name="ElementFinder",
        iterator=lambda input: finder.invoke(input),
        step_name_getter=attrgetter("name"),
        content_getter=attrgetter("content"),
        is_last_getter=attrgetter("is_last"),
    )
