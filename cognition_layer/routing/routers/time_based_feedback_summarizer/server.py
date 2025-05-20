from operator import attrgetter

from cognition_layer.api import FastAgentProtocol
from cognition_layer.routing.routers.time_based_feedback_summarizer.router import (
    TimeBasedFeedbackSummarizer,
)


def get_fast_ap_server(*args, **kwargs) -> FastAgentProtocol:

    router = TimeBasedFeedbackSummarizer(*args, **kwargs)
    return FastAgentProtocol(
        name="TimeBasedFeedbackSummarizer Server",
        iterator=lambda input: router.invoke(input),
        step_name_getter=attrgetter("name"),
        content_getter=attrgetter("content"),
        is_last_getter=attrgetter("is_last"),
    )
