import sys
from pathlib import Path

import colorlog
import dotenv

from ecm.constants import LOG_LEVEL


class _MOCKS_ENABLED:
    status: bool = False


def enable_mocks():
    _MOCKS_ENABLED.status = True


if "pytest" in sys.modules:
    enable_mocks()


def default_kwargs(defaults: dict, new_kwargs: dict):
    """Returns a dictionary containing the default kwargs
    with the new_kwargs replaced if any."""
    defaults.update(new_kwargs)
    return defaults


def get_root_path() -> Path:
    """
    Retrieves the path to the root directory of the repository.
    :return: The Path object pointing to the core directory.
    :rtype: Path
    """
    # ----------------------------------------------------------------
    # CAUTION: Moving this function to a different file or changing
    # the location of the file can shift the result!!!
    # ----------------------------------------------------------------

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


def load_env():
    root = get_root_path()
    paths = [
        root / ".env",
        root / "action_space" / "experimental" / "mouse" / ".env",
        root / "ecm" / "remote" / ".env",
    ]

    for path in paths:
        dotenv.load_dotenv(path, override=False)


load_env()
