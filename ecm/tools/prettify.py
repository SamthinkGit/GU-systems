import json
import traceback
from pathlib import Path
from pprint import pformat
from typing import Any
from typing import Optional

from colorama import Fore
from colorama import Style

from ecm.shared import get_root_path


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


def _color_by_type(name, node_type):
    if node_type == "router":
        return Fore.BLUE + "📡  " + name + Style.RESET_ALL
    elif node_type == "agent":
        return Fore.GREEN + "🛠️  " + name + Style.RESET_ALL
    elif node_type == "cluster":
        return Fore.GREEN + "👥  " + name + Style.RESET_ALL
    else:
        return name + f" [{node_type}]"


def _print_graph(node, prefix="", is_root=True):
    name = node.get("name", "Unnamed")
    node_type = node.get("type", "unknown")

    if is_root:
        print(_color_by_type(name, node_type))

    workers = node.get("workers", [])
    for i, child in enumerate(workers):
        is_last = i == len(workers) - 1
        branch = "└── " if is_last else "├── "
        child_prefix = prefix + ("    " if is_last else "│   ")

        print(
            prefix
            + branch
            + _color_by_type(child["name"], child.get("type", "unknown"))
        )
        _print_graph(child, prefix=child_prefix, is_root=False)


def pretty_print_schema(id: str = None, path: Optional[str] = None):
    if path is None:
        path = (
            get_root_path() / "cognition_layer" / "routing" / "schemas" / f"{id}.json"
        )
    else:
        path = Path(path)
        id = path.name

    graph = json.loads(path.read_text())

    print(pretty_head(id))
    print("\n")
    _print_graph(graph)
    print("\n")
    print(f"{Style.BRIGHT}{'=' * (42 + len(id))}{Style.RESET_ALL}")


def truncate_with_ellipsis(text: str, max_length: int) -> str:
    text = str(text)
    if len(text) <= max_length:
        return text
    return text[: max_length - 3] + "..."
