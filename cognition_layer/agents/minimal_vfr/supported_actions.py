# flake8: noqa
def load_darkvfr_supported_actions() -> None:
    """Load the supported/recommended actions for the DarkVFR agent."""
    import action_space.experimental.screenshot.actions
    import action_space.keyboard.actions
    import action_space.meta.cognition_state.actions
    import action_space.meta.fake.actions
    import action_space.mouse.molmo_based.actions
    import action_space.vision.moondream.actions
