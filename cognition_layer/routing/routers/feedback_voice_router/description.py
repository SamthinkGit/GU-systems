# flake8: noqa


def server_loader():
    from cognition_layer.routing.routers.feedback_voice_router.server import (
        get_fast_ap_server,
    )

    return get_fast_ap_server


DEPLOY_MODEL = {
    "name": "FeedbackVoiceRouter",
    "alias": ["feedback-voice-router", "feedbackvoicerouter"],
    "agent_description": "A generator that produces natural spoken phrases simulating the execution of described actions.",
    "response_description": "Generating a short spoken sentence that simulates the described action.",
    "use_case": "Voice feedback generation for voice assistants or user-facing systems with spoken interaction.",
    "type": "router",
    "packages": [],
    "server": server_loader,
}
