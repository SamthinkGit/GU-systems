# flake8: noqa


def server_loader():
    from cognition_layer.routing.routers.zero_shot_router.server import (
        get_fast_ap_server,
    )

    return get_fast_ap_server


DEPLOY_MODEL = {
    "name": "ZeroShotRouter",
    "alias": ["zero-shot-router", "zsr", "zeroshotrouter"],
    "agent_description": "A router that uses zero-shot to redirect requests",
    "response_description": "Routing to the appropriate agent",
    "use_case": "",
    "type": "router",
    "packages": [],
    "server": server_loader,
}
