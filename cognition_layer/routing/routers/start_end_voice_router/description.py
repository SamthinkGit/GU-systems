# flake8: noqa


def server_loader():
    from cognition_layer.routing.routers.start_end_voice_router.server import (
        get_fast_ap_server,
    )

    return get_fast_ap_server


DEPLOY_MODEL = {
    "name": "StartEndVoiceRouter",
    "alias": ["start-end-voice-router", "startendvoicerouter"],
    "agent_description": "A generator that produces natural spoken phrases for the start and end of the task.",
    "response_description": "Subnodes",
    "use_case": "Voice feedback generation for starting/ending tasks",
    "type": "router",
    "packages": [],
    "server": server_loader,
}
