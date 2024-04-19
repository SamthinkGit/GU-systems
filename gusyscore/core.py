import warnings
from pathlib import Path

import colorlog

from gusyscore.constants import LOG_LEVEL


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


def get_logger(name: str):

    handler = colorlog.StreamHandler()
    formatter = colorlog.ColoredFormatter(
        "[%(log_color)s%(levelname)s%(reset)s] [%(yellow)s%(name)s%(reset)s]: %(message)s",
        log_colors={
            "DEBUG": "cyan",
            "INFO": "green",
            "WARNING": "purple",
            "ERROR": "red",
            "CRITICAL": "red,bg_white",
        },
        secondary_log_colors={},
        style="%",
    )

    handler.setFormatter(formatter)
    logger = colorlog.getLogger(name)
    logger.setLevel(LOG_LEVEL)
    logger.addHandler(handler)
    return logger


def ignore_invalid_warnings():
    invalid_msgs = [
        'Two goals were accepted with the same ID'
    ]
    for msg in invalid_msgs:
        warnings.filterwarnings("ignore", message=msg)
