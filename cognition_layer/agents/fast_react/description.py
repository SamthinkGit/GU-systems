# flake8: noqa
def server_loader():
    from cognition_layer.agents.fast_react.api.server import (
        get_fast_ap_server,
    )
    return get_fast_ap_server


def load_fastreact_supported_actions() -> None:
    """Load the supported/recommended actions for the FastReact agent."""

    import action_space.keyboard.actions
    import action_space.experimental.screenshot.actions
    import action_space.mouse.labelled_ocr.actions


DEPLOY_MODEL = {
    "name": "fastreact",
    "alias": ["fast-react", "fr"],
    "agent_description": "An autonomous agent capable of solving complex tasks on the user environment.",
    "response_description": "The reasoning about the actions running/completed",
    "use_case": "Solve a complex task or interact with the computer/environment",
    "type": "agent",
    "supported_actions": load_fastreact_supported_actions,
    "server": server_loader,
}
