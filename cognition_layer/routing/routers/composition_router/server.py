from operator import attrgetter

from cognition_layer.api import FastAgentProtocol
from cognition_layer.routing.routers.composition_router.router import CompositionRouter


def get_fast_ap_server(*args, **kwargs) -> FastAgentProtocol:

    router = CompositionRouter(*args, **kwargs)
    return FastAgentProtocol(
        name="CompositionRouter Server",
        iterator=lambda input: router.invoke(input),
        step_name_getter=attrgetter("name"),
        content_getter=attrgetter("content"),
        is_last_getter=attrgetter("is_last"),
    )
