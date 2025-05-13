# flake8: noqa
def server_loader():
    from cognition_layer.agents.visual_fast_react.api.server import (
        get_fast_ap_server,
    )

    return get_fast_ap_server


DEPLOY_MODEL = {
    "name": "VisualFastReact",
    "alias": ["visual-fast-react", "vfr"],
    "agent_description": "An autonomous agent capable of solving complex tasks on the user environment.",
    "response_description": "The reasoning about the actions running/completed",
    "use_case": "Solve a complex task or interact with the computer/environment",
    "type": "agent",
    "packages": ["screenshot", "keyboard", "labelled_ocr_mouse_actions"],
    "server": server_loader,
}
