import os

from ecm.shared import get_logger


def clear_terminal(reason: str = ""):
    """Clear the terminal screen."""
    os.system("cls" if os.name == "nt" else "clear")
    _logger = get_logger("ActionSpace")
    reason = f"Reason: {reason} " if reason else ""
    _logger.debug(f"Terminal cleared. {reason}")
