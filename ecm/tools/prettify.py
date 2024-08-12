import traceback
from pprint import pformat
from typing import Any

from colorama import Fore
from colorama import Style


def pretty_print(value: Any):
    try:
        print(_prettify_dict(dict(value)))
        return
    except Exception:
        raise SystemError(f"Value {value} is not a valid type for prettify.\n" + traceback.format_exc())


def _prettify_dict(dict: dict[Any, Any]) -> str:
    return "\n".join(
        [
            f"{Fore.YELLOW + Style.BRIGHT}{key}: {Style.RESET_ALL}{pformat(value)}"
            for key, value in dict.items()
        ]
    )
