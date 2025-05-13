# flake8: noqa
def server_loader():
    from cognition_layer.agents.minimal_vfr.api.darkvfr_server import (
        get_fast_ap_server,
    )

    return get_fast_ap_server


DEPLOY_MODEL = {
    "name": "DarkVFR",
    "alias": ["dark-vfr", "darkvfr", "dvfr"],
    "agent_description": "An autonomous agent capable of solving complex tasks on the user environment.",
    "response_description": "The reasoning about the actions running/completed",
    "use_case": "Solve a complex task or interact with the computer/environment",
    "type": "agent",
    "packages": [
        "screenshot",
        "keyboard",
        "sleep",
        "apps-management",
        "moondream_vision_actions",
        "molmo_mouse_actions",
        "meta-look",
        "meta",
    ],
    "server": server_loader,
}
