from operator import attrgetter

from cognition_layer.api import FastAgentProtocol
from cognition_layer.experts.planner.agent import PlannerExpert


def get_fast_ap_server(*args, **kwargs) -> FastAgentProtocol:

    planner = PlannerExpert(*args, **kwargs)
    return FastAgentProtocol(
        name="ExpertPlanner",
        iterator=lambda input: planner.invoke(input),
        step_name_getter=attrgetter("name"),
        content_getter=attrgetter("content"),
        is_last_getter=attrgetter("is_last"),
    )
