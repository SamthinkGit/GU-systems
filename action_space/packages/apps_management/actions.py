import ctypes
import json
import os
import subprocess
from functools import cache
from pathlib import Path
from typing import Literal

import psutil
import pyautogui as pag

from cognition_layer.tools.nlp.similarity import word_similarity
from ecm.shared import get_root_path
from ecm.tools.get_platform import osmethod
from ecm.tools.item_registry_v2 import ItemRegistry

PKG_NAME = "apps-management"

VALID_EXECUTABLES = [".exe", ".lnk"]
MIN_WINDOW_NAME_SIMILARITY = 0.7
SLEEP_TIME_BETWEEN_CHECKS = 1
MAX_SECONDS_TO_OPEN_APP = 15


# ========================= ACTIONS ======================
@ItemRegistry.register(type="action", package=PKG_NAME)
def get_opened_windows() -> str:
    """
    Returns a list of all opened windows and its names on the current machine.
    Additionally, it informs if the window is focused or not.
    Often the name is replaced by the content of the window (For example, the browser could show the name of the page).
    It is a valid name.
    """
    return "\n".join([str(info) for info in _get_opened_windows()])


@ItemRegistry.register(type="action", package=PKG_NAME)
def activate_already_opened_window(name: str) -> str:
    """
    After knowing the name of an opened window, activate it and maximize in the screen.
    The name should be the same as the one shown in the 'get_opened_windows' action.
    Please don't confuse with the path of the executable.
    Example: activate_already_open_window('Program Manager')
    """
    return _activate_already_opened_window(name)


@ItemRegistry.register(type="action", package=PKG_NAME)
def search_for_installed_app(name: str) -> str:
    """
    Search for an installed app in the system and the path of its executables.
    Use me when the app is not already opened and you want to know the path to open it.
    Example: search_for_installed_app('Edge')
    """
    return _search_for_installed_app(name)


@ItemRegistry.register(type="action", package=PKG_NAME)
def open_app_from_path(name: str) -> str:
    """
    Opens an app from its path.
    Please first retrieve the path using 'search_for_installed_app' action.
    Use me when you know the path of the app you want to open.
    Please use '/' as separator in the path.
    Note: It is recommended to check the opened windows and activate the app after this action.
    Example: open_app_from_path('C:/Program Files/Microsoft/Edge/Application/msedge.exe')
    """
    return _open_app_from_path(name)


# ========================= IMPLEMENTATION ======================
def _get_opened_windows() -> list[dict]:
    windows = pag.getAllWindows()
    info = []
    for win in windows:
        if not win.title.strip():
            continue
        win_info = {
            "name": win.title,
            "is_active": win.isActive,
            "path": _get_sanitized_path(
                _get_executable_path_from_hwnd(win._hWnd),
            ),
        }
        info.append(win_info)

    return info


def _activate_already_opened_window(name: str) -> str:
    windows = pag.getAllWindows()
    sorted_windows = list(
        sorted(windows, key=lambda x: word_similarity(x.title, name), reverse=True)
    )
    win = (
        sorted_windows[0]
        if word_similarity(sorted_windows[0].title, name) > MIN_WINDOW_NAME_SIMILARITY
        else None
    )

    if win is None:
        return f"Window with name '{name}' not found. Plase check 'get_opened_windows' action to see the available windows."  # noqa

    if not win.isActive:
        pag.press("altleft")
        win.activate()
        win.maximize()

    return f"Window with name '{win.title}' activated and maximized."  # noqa


@osmethod
def _search_for_installed_app(
    os: Literal["Windows", "Raspbian", "Ubuntu", "Uknown"], name: str
) -> str:

    if os == "Windows":
        search_tool_path = (
            get_root_path()
            / "action_space"
            / "packages"
            / "apps_management"
            / "windows"
            / "WindowsSearchWrapper.exe"
        )
        output = subprocess.run(
            [search_tool_path, name],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        paths: list[str] = json.loads(output.stdout)
        valid_paths = []
        for path in paths:
            if Path(path).suffix in VALID_EXECUTABLES:
                valid_paths.append(_get_sanitized_path(path))
        return "\n".join(valid_paths)
    else:
        raise NotImplementedError()


def _open_app_from_path(name: str) -> str:
    name = _get_sanitized_path(name)
    if Path(name).exists():
        os.startfile(name)
        return f"App with path '{name}' is trying to be opened."  # noqa
    else:
        return f"Path '{name}' not found. Please check the path and try again."  # noqa


@cache
def _get_win_ctypes():
    user32 = ctypes.windll.user32
    kernel32 = ctypes.windll.kernel32
    psapi = ctypes.windll.psapi
    return user32, kernel32, psapi


def _get_executable_path_from_hwnd(hwnd):
    user32, kernel32, psapi = _get_win_ctypes()

    pid = ctypes.c_ulong()
    user32.GetWindowThreadProcessId(hwnd, ctypes.byref(pid))
    process_id = pid.value

    try:
        process = psutil.Process(process_id)
        return process.exe()  # Ruta del ejecutable
    except psutil.NoSuchProcess:
        return None


def _get_sanitized_path(path: str) -> str:
    """Sanitizes the path by replacing backslashes with forward slashes."""
    return path.replace("\\", "/")
