# flake8: noqa
def load_fastreact_supported_actions() -> None:
    """Load the supported/recommended actions for the FastReact agent."""

    import action_space.keyboard.actions
    import action_space.experimental.screenshot.actions
    import action_space.mouse.labelled_ocr.actions
