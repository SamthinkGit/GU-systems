# flake8: noqa
def load_vfr_supported_actions() -> None:
    """Load the supported/recommended actions for the VisualFastReact agent."""

    import action_space.keyboard.actions
    import action_space.experimental.screenshot.actions
    import action_space.mouse.labelled_ocr.actions
