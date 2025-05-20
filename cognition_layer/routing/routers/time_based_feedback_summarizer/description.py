# flake8: noqa


def server_loader():
    from cognition_layer.routing.routers.time_based_feedback_summarizer.server import (
        get_fast_ap_server,
    )

    return get_fast_ap_server


DEPLOY_MODEL = {
    "name": "TimeBasedFeedbackSummarizer",
    "alias": ["time-based-feedback-summarizer"],
    "agent_description": "A router that summarizes feedback each <delay> seconds.",
    "response_description": "Summaries of the workers' feedback.",
    "use_case": "Time-based feedback summarization",
    "type": "router",
    "packages": [],
    "server": server_loader,
}
