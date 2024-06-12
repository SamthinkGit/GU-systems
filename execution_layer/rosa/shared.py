import warnings
from pathlib import Path


def get_rosa_path() -> Path:
    """
    Retrieves the path to the core directory of the application, ensuring it resides within the expected
    'rosa' directory.
    :return: The Path object pointing to the core directory.
    :rtype: Path
    :raises AssertionError: If the resolved path does not end with 'core', indicating an unexpected
    or unsafe directory structure.
    """

    path = Path(__file__).parent.resolve()
    assert str(path).endswith(
        "rosa"
    ), f"Path resolved with get_core_path is not safe: {path}"
    return path


def ignore_invalid_warnings():
    invalid_msgs = ["Two goals were accepted with the same ID"]
    for msg in invalid_msgs:
        warnings.filterwarnings("ignore", message=msg)
