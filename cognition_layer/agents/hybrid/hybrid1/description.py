# flake8: noqa
def server_loader():
    from cognition_layer.agents.hybrid.hybrid1.server import (
        get_fast_ap_server,
    )

    return get_fast_ap_server


DEPLOY_MODEL = {
    "name": "Hybrid1",
    "alias": ["hybrid1", "hybrid-1"],
    "agent_description": "An autonomous agent capable of solving complex tasks on the user environment.",
    "response_description": "The reasoning about the actions running/completed",
    "use_case": "Interact via commands/clicks or keyboard with the computer",
    "type": "agent",
    "packages": [
        "expert-calling",
        "sleep",
        "meta",
    ],
    "server": server_loader,
}
