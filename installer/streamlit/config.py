from pathlib import Path

LATEST_COGNITION_LAYER = "DarkVFR"
LATEST_EXECUTION_LAYER = "Pyxcel"

ECM_OPTIONS = ["base", "devel"]
EXECUTION_LAYER_OPTIONS = ["Pyxcel", "All"]
COGNITION_LAYER_OPTIONS = ["DarkVFR", "VisualFastReact", "FastReact", "Base", "All"]


def get_repository_root_path() -> Path:
    """
    Get the root path of the repository.
    """
    return Path(__file__).resolve().parent.parent.parent
