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
    "use_case": "Interact via commands/clicks or keyboard with the computer",
    "type": "agent",
    "packages": [
        "screenshot",
        "keyboard",
        "sleep",
        "apps-management",
        "moondream_vision_actions",
        "molmo_vision_actions",
        "simple-read-ocr",
        "spotlight",
        "mouse-simple",
        "meta-look",
        "meta",
    ],
    "server": server_loader,
}
