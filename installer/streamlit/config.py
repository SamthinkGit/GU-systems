from pathlib import Path

LATEST_COGNITION_LAYER = "DarkVFR"
LATEST_EXECUTION_LAYER = "Pyxcel"

EXECUTION_LAYER_OPTIONS = ["Pyxcel", "All"]
COGNITION_LAYER_OPTIONS = ["DarkVFR", "VisualFastReact", "FastReact", "All"]


def get_repository_root_path() -> Path:
    """
    Get the root path of the repository.
    """
    return Path(__file__).resolve().parent.parent.parent
