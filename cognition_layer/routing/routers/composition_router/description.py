# flake8: noqa


def server_loader():
    from cognition_layer.routing.routers.composition_router.server import (
        get_fast_ap_server,
    )

    return get_fast_ap_server


DEPLOY_MODEL = {
    "name": "CompositionRouter",
    "alias": ["composition-router"],
    "agent_description": "A router that chains calls to specialized agents until completing the task",
    "response_description": "Reasoning and calls to the appropriate agents",
    "use_case": "Chaining calls to specialized agents",
    "type": "router",
    "packages": [],
    "server": server_loader,
}
