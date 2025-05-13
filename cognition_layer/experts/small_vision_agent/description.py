# flake8: noqa


def server_loader():
    from cognition_layer.experts.small_vision_agent.api.server import (
        get_fast_ap_server,
    )

    return get_fast_ap_server


DEPLOY_MODEL = {
    "name": "SmallVisionAgent",
    "alias": ["small-vision-agent", "sva"],
    "agent_description": "A small vision agent that can process images and respond to queries.",
    "response_description": "Short response to the user",
    "use_case": "Use it only to give responses to the user based on the display/screenshot.",
    "type": "agent",
    "packages": ["screenshot"],
    "server": server_loader,
}
