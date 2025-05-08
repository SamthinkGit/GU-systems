# flake8: noqa
def server_loader():
    from cognition_layer.agents.minimal_vfr.api.darkvfr_server import (
        get_fast_ap_server,
    )
    return get_fast_ap_server

def load_darkvfr_supported_actions() -> None:
    """Load the supported/recommended actions for the DarkVFR agent."""
    import action_space.experimental.screenshot.actions
    import action_space.keyboard.actions
    import action_space.meta.cognition_state.actions
    import action_space.meta.fake.actions
    import action_space.mouse.molmo_based.actions
    import action_space.vision.moondream.actions
    import action_space.packages.apps_management.actions
    import action_space.packages.sleep.actions


DEPLOY_MODEL = {
    "name": "DarkVFR",
    "alias": ["dark-vfr", "dvfr"],
    "agent_description": "An autonomous agent capable of solving complex tasks on the user environment.",
    "response_description": "The reasoning about the actions running/completed",
    "use_case": "Solve a complex task or interact with the computer/environment",
    "type": "agent",
    "supported_actions": load_darkvfr_supported_actions,
    "server": server_loader,
}
