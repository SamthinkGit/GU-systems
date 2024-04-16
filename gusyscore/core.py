from pathlib import Path


def get_core_path() -> Path:
    """
    Retrieves the path to the core directory of the application, ensuring it resides within the expected
    'gusyscore' directory.
    :return: The Path object pointing to the core directory.
    :rtype: Path
    :raises AssertionError: If the resolved path does not end with 'gusyscore', indicating an unexpected
    or unsafe directory structure.
    """

    path = Path(__file__).parent.resolve()
    assert str(path).endswith(
        "gusyscore"
    ), f"Path resolved with get_core_path is not safe: {path}"
    return path


def get_root_path() -> Path:
    """
    Retrieves the path to the root directory of the application.
    :return: The Path object pointing to the core directory.
    :rtype: Path
    """

    path = Path(__file__).parent.parent.resolve()
    return path
