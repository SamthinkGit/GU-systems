import importlib
import pickle
import re
from functools import cache
from pathlib import Path
from typing import Any
from typing import Dict

from ecm.shared import get_root_path

ACTIONS_FILE_NAME = "actions.py"


@cache
def discover_packages(
    root_path: Path = get_root_path() / "action_space",
    static: bool = False,
) -> Dict[str, Dict[str, str]]:
    """
    Recursively scans the directory 'root_path' to find internal packages.
    A package is identified by containing a file 'actions.py' with a variable PKG_NAME.
    Returns a dictionary where the key is the package notation (e.g., 'meta/example')
    and the value is another dictionary with:
      - 'notation': the notation of the package (e.g., 'meta/example')
      - 'pkg_name': the name of the package (PKG_NAME)
      - 'module_path': module path to import (e.g., 'action_space.meta.example.actions')
    """
    if static:
        with open(
            get_root_path() / "action_space" / "static" / "packages_index.pkl", "rb"
        ) as f:
            return pickle.load(f)
    packages = []
    for actions_file in root_path.rglob(ACTIONS_FILE_NAME):
        content = actions_file.read_text(encoding="utf-8")
        m = re.search(r"^PKG_NAME\s*=\s*[\'\"]([^\'\"]+)[\'\"]", content, re.MULTILINE)
        if not m:
            continue
        pkg_name = m.group(1)
        rel_dir = actions_file.parent.relative_to(root_path)
        notation = str(rel_dir).replace("\\", "/").replace("/", "/")
        module_path = ".".join(
            actions_file.with_suffix("").relative_to(root_path).parts
        )
        packages.append(
            {"pkg_name": pkg_name, "module_path": module_path, "notation": notation}
        )
    validate_unique_packages(packages)
    return packages


def load_package(
    identifier: str,
    root_path: Path = get_root_path() / "action_space",
    pkg_import_path: str = "action_space",
    static: bool = False,
) -> Any:
    """
    Dynamically loads an internal package given its notation or name (PKG_NAME).
    - identifier: can be the notation ('meta/example') or the PKG_NAME.
    - root_path: root directory containing the 'action_space' folder.
    Returns the 'actions' module of the package.
    """
    packages = discover_packages(root_path, static=static)
    info = False
    for pkg in packages:
        if pkg["notation"] == identifier or pkg["pkg_name"] == identifier:
            info = pkg
            break
    if not info:
        raise ValueError(
            f"""
            Package '{identifier}' not found. Current packages: {list(packages)}.
            Posible solutions:
            - Generate a new static package index with `generate_static_package_index()` if using static mode.
            - Ensure the package exists in the 'action_space' directory.
            """
        )

    import_path = pkg_import_path + "." + info["module_path"]
    importlib.import_module(import_path)


def validate_unique_packages(packages: list) -> None:
    """
    Validates that there are no duplicated package notations or PKG_NAME values.
    Raises a ValueError if any duplicates are found.
    """
    seen_pkg_names = {}
    for pkg in packages:
        pkg_name = pkg["pkg_name"]
        if pkg_name in seen_pkg_names:
            raise ValueError(
                f"Duplicate PKG_NAME '{pkg_name}' found in '{pkg['notation']}' and '{seen_pkg_names[pkg_name]}'"
            )
        seen_pkg_names[pkg_name] = pkg["notation"]


def generate_static_package_index(
    root_path: Path = get_root_path() / "action_space",
    output_path: Path = get_root_path()
    / "action_space"
    / "static"
    / "packages_index.pkl",
):
    """
    Generates and stores the package index as a static pickle file.
    """
    data = discover_packages(root_path=root_path, static=False)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, "wb") as f:
        pickle.dump(data, f)
    print(f"✅ Static package index generated at: {output_path}")
