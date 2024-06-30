from operator import attrgetter

from cognition_layer.api import ServerAPI
from cognition_layer.planexv2.agents.planexv2 import PlanexV2
from cognition_layer.tools.server_template import ServerFromIterator


def get_server(verbose: bool = False) -> ServerAPI:
    agent = PlanexV2()
    return ServerFromIterator(
        name="PlanexV2",
        iterator=lambda input: agent.iter(input, verbose=verbose),
        step_name_getter=attrgetter("next_agent"),
        content_getter=attrgetter("content"),
        is_last_getter=attrgetter("is_last"),
    )
