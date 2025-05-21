from operator import attrgetter

from cognition_layer.agents.hybrid.hybrid1.agent import Hybrid1
from cognition_layer.api import FastAgentProtocol


def get_fast_ap_server(*args, **kwargs) -> FastAgentProtocol:

    hybrid = Hybrid1(*args, **kwargs)
    return FastAgentProtocol(
        name="Hybrid1 Server",
        iterator=lambda input: hybrid.complete_task(input),
        step_name_getter=attrgetter("name"),
        content_getter=attrgetter("content"),
        is_last_getter=attrgetter("is_last"),
    )
