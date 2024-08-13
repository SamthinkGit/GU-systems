import traceback
from pprint import pformat
from typing import Any
from typing import Optional

from colorama import Fore
from colorama import Style


def pretty_print(value: Any, header: Optional[str] = None):

    if header:
        print(pretty_head(header))

    try:
        print(_prettify_dict(dict(value)))
        return
    except Exception:
        raise SystemError(
            f"Value {value} is not a valid type for prettify.\n"
            + traceback.format_exc()
        )


def pretty_head(label: str):
    return f"{Style.BRIGHT}{'=' * 20} {label} {'=' * 20}{Style.RESET_ALL}"


def _prettify_dict(dict: dict[Any, Any]) -> str:
    return "\n".join(
        [
            f"{Fore.YELLOW + Style.BRIGHT}{key}: {Style.RESET_ALL}{pformat(value)}"
            for key, value in dict.items()
        ]
    )
