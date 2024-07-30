# Copyright 2015 The Chromium OS Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.
# Get all evdev/uinput events in our namespace.
# pylint: disable=wildcard-import, unused-wildcard-import
import subprocess
import time

import uinput

from ecm.tools.registry import ItemRegistry

# Time to wait after uinput device creation, before starting to type. This is
# needed as kernel/Chrome takes some time to register the new input device.
#
# TODO(crbug.com/714950): This is a hack, we should figure out a way to check
# when kernel/Chrome is ready (monitor udev events?), instead of waiting for
# an arbitrary amount of time.
STARTUP_DELAY = 0.2
# Default delay between key presses in seconds. 12ms is the xdotool default.
DEFAULT_DELAY = 0.012
uinput_device_keyboard = None
# This dictionary contains most 7 bit ASCII characters. Add more if needed.
# TODO(ihf): Create this table using xkbcommon to support arbirtrary
# character sets and keyboard layouts.
_CROS_CHAR_MAP = {
    "\b": [uinput.KEY_BACKSPACE],
    "\t": [uinput.KEY_TAB],
    "\n": [uinput.KEY_ENTER],
    " ": [uinput.KEY_SPACE],
    "!": [uinput.KEY_LEFTSHIFT, uinput.KEY_1],
    '"': [uinput.KEY_LEFTSHIFT, uinput.KEY_APOSTROPHE],
    "#": [uinput.KEY_LEFTSHIFT, uinput.KEY_3],
    "$": [uinput.KEY_LEFTSHIFT, uinput.KEY_4],
    "%": [uinput.KEY_LEFTSHIFT, uinput.KEY_5],
    "&": [uinput.KEY_LEFTSHIFT, uinput.KEY_7],
    "'": [uinput.KEY_APOSTROPHE],
    "(": [uinput.KEY_LEFTSHIFT, uinput.KEY_9],
    ")": [uinput.KEY_LEFTSHIFT, uinput.KEY_0],
    "*": [uinput.KEY_KPASTERISK],
    "+": [uinput.KEY_LEFTSHIFT, uinput.KEY_EQUAL],
    ",": [uinput.KEY_COMMA],
    "-": [uinput.KEY_MINUS],
    ".": [uinput.KEY_DOT],
    "/": [uinput.KEY_SLASH],
    "0": [uinput.KEY_0],
    "1": [uinput.KEY_1],
    "2": [uinput.KEY_2],
    "3": [uinput.KEY_3],
    "4": [uinput.KEY_4],
    "5": [uinput.KEY_5],
    "6": [uinput.KEY_6],
    "7": [uinput.KEY_7],
    "8": [uinput.KEY_8],
    "9": [uinput.KEY_9],
    ":": [uinput.KEY_LEFTSHIFT, uinput.KEY_SEMICOLON],
    ";": [uinput.KEY_SEMICOLON],
    "<": [uinput.KEY_LEFTSHIFT, uinput.KEY_COMMA],
    "=": [uinput.KEY_EQUAL],
    ">": [uinput.KEY_LEFTSHIFT, uinput.KEY_DOT],
    "?": [uinput.KEY_LEFTSHIFT, uinput.KEY_SLASH],
    "@": [uinput.KEY_LEFTSHIFT, uinput.KEY_2],
    "A": [uinput.KEY_LEFTSHIFT, uinput.KEY_A],
    "B": [uinput.KEY_LEFTSHIFT, uinput.KEY_B],
    "C": [uinput.KEY_LEFTSHIFT, uinput.KEY_C],
    "D": [uinput.KEY_LEFTSHIFT, uinput.KEY_D],
    "E": [uinput.KEY_LEFTSHIFT, uinput.KEY_E],
    "F": [uinput.KEY_LEFTSHIFT, uinput.KEY_F],
    "G": [uinput.KEY_LEFTSHIFT, uinput.KEY_G],
    "H": [uinput.KEY_LEFTSHIFT, uinput.KEY_H],
    "I": [uinput.KEY_LEFTSHIFT, uinput.KEY_I],
    "J": [uinput.KEY_LEFTSHIFT, uinput.KEY_J],
    "K": [uinput.KEY_LEFTSHIFT, uinput.KEY_K],
    "L": [uinput.KEY_LEFTSHIFT, uinput.KEY_L],
    "M": [uinput.KEY_LEFTSHIFT, uinput.KEY_M],
    "N": [uinput.KEY_LEFTSHIFT, uinput.KEY_N],
    "O": [uinput.KEY_LEFTSHIFT, uinput.KEY_O],
    "P": [uinput.KEY_LEFTSHIFT, uinput.KEY_P],
    "Q": [uinput.KEY_LEFTSHIFT, uinput.KEY_Q],
    "R": [uinput.KEY_LEFTSHIFT, uinput.KEY_R],
    "S": [uinput.KEY_LEFTSHIFT, uinput.KEY_S],
    "T": [uinput.KEY_LEFTSHIFT, uinput.KEY_T],
    "U": [uinput.KEY_LEFTSHIFT, uinput.KEY_U],
    "V": [uinput.KEY_LEFTSHIFT, uinput.KEY_V],
    "W": [uinput.KEY_LEFTSHIFT, uinput.KEY_W],
    "X": [uinput.KEY_LEFTSHIFT, uinput.KEY_X],
    "Y": [uinput.KEY_LEFTSHIFT, uinput.KEY_Y],
    "Z": [uinput.KEY_LEFTSHIFT, uinput.KEY_Z],
    "[": [uinput.KEY_LEFTBRACE],
    "\\": [uinput.KEY_BACKSLASH],
    "]": [uinput.KEY_RIGHTBRACE],
    "^": [uinput.KEY_LEFTSHIFT, uinput.KEY_6],
    "_": [uinput.KEY_LEFTSHIFT, uinput.KEY_MINUS],
    "`": [uinput.KEY_GRAVE],
    "a": [uinput.KEY_A],
    "b": [uinput.KEY_B],
    "c": [uinput.KEY_C],
    "d": [uinput.KEY_D],
    "e": [uinput.KEY_E],
    "f": [uinput.KEY_F],
    "g": [uinput.KEY_G],
    "h": [uinput.KEY_H],
    "i": [uinput.KEY_I],
    "j": [uinput.KEY_J],
    "k": [uinput.KEY_K],
    "l": [uinput.KEY_L],
    "m": [uinput.KEY_M],
    "n": [uinput.KEY_N],
    "o": [uinput.KEY_O],
    "p": [uinput.KEY_P],
    "q": [uinput.KEY_Q],
    "r": [uinput.KEY_R],
    "s": [uinput.KEY_S],
    "t": [uinput.KEY_T],
    "u": [uinput.KEY_U],
    "v": [uinput.KEY_V],
    "w": [uinput.KEY_W],
    "x": [uinput.KEY_X],
    "y": [uinput.KEY_Y],
    "z": [uinput.KEY_Z],
    "{": [uinput.KEY_LEFTSHIFT, uinput.KEY_LEFTBRACE],
    "|": [uinput.KEY_LEFTSHIFT, uinput.KEY_BACKSLASH],
    "}": [uinput.KEY_LEFTSHIFT, uinput.KEY_RIGHTBRACE],
    "~": [uinput.KEY_LEFTSHIFT, uinput.KEY_GRAVE],
}
# A list of American English ChromeOS keys to define a keyboard device.
_CROS_KEYS_ALL = [
    # Function row.
    uinput.KEY_ESC,
    uinput.KEY_F1,
    uinput.KEY_F2,
    uinput.KEY_F3,
    uinput.KEY_F4,
    uinput.KEY_F5,
    uinput.KEY_F6,
    uinput.KEY_F7,
    uinput.KEY_F8,
    uinput.KEY_F9,
    uinput.KEY_F10,
    uinput.KEY_F11,
    uinput.KEY_F12,
    uinput.KEY_HOME,
    uinput.KEY_END,
    uinput.KEY_INSERT,
    uinput.KEY_DELETE,
    # First row.
    uinput.KEY_GRAVE,
    uinput.KEY_1,
    uinput.KEY_2,
    uinput.KEY_3,
    uinput.KEY_4,
    uinput.KEY_5,
    uinput.KEY_6,
    uinput.KEY_7,
    uinput.KEY_8,
    uinput.KEY_9,
    uinput.KEY_0,
    uinput.KEY_MINUS,
    uinput.KEY_EQUAL,
    uinput.KEY_BACKSPACE,
    # Second row.
    uinput.KEY_TAB,
    uinput.KEY_Q,
    uinput.KEY_W,
    uinput.KEY_E,
    uinput.KEY_R,
    uinput.KEY_T,
    uinput.KEY_Y,
    uinput.KEY_U,
    uinput.KEY_I,
    uinput.KEY_O,
    uinput.KEY_P,
    uinput.KEY_LEFTBRACE,
    uinput.KEY_RIGHTBRACE,
    uinput.KEY_BACKSLASH,
    # Third row
    uinput.KEY_CAPSLOCK,
    uinput.KEY_A,
    uinput.KEY_S,
    uinput.KEY_D,
    uinput.KEY_F,
    uinput.KEY_G,
    uinput.KEY_H,
    uinput.KEY_J,
    uinput.KEY_K,
    uinput.KEY_L,
    uinput.KEY_SEMICOLON,
    uinput.KEY_APOSTROPHE,
    uinput.KEY_ENTER,
    # Forth row.
    uinput.KEY_LEFTSHIFT,
    uinput.KEY_Z,
    uinput.KEY_X,
    uinput.KEY_C,
    uinput.KEY_V,
    uinput.KEY_B,
    uinput.KEY_N,
    uinput.KEY_M,
    uinput.KEY_COMMA,
    uinput.KEY_DOT,
    uinput.KEY_SLASH,
    uinput.KEY_RIGHTSHIFT,
    # Fifth row.
    uinput.KEY_LEFTCTRL,
    uinput.KEY_FN,
    uinput.KEY_SEARCH,
    uinput.KEY_LEFTALT,
    uinput.KEY_SPACE,
    uinput.KEY_NUMLOCK,
    uinput.KEY_SCROLLLOCK,
    uinput.KEY_RIGHTALT,
    uinput.KEY_RIGHTCTRL,
    # Directional keys.
    uinput.KEY_UP,
    uinput.KEY_PAGEUP,
    uinput.KEY_LEFT,
    uinput.KEY_RIGHT,
    uinput.KEY_DOWN,
    uinput.KEY_PAGEDOWN,
]


def _chars_to_events(chars):
    """
    Translates string to key events.
    @param chars: characters to translate to events.
    @returns: list of lists of events representing characters.
    """
    events = []
    for char in chars:
        events.append(_CROS_CHAR_MAP[char])
    return events


def _get_uinput_device_keyboard():
    """
    Lazy initialize device and return it. We don't want to create a device
    during build_packages or for tests that don't need it, hence init with None.
    """
    global uinput_device_keyboard
    if uinput_device_keyboard is None:
        # For DUTs without keyboard attached force load uinput.
        subprocess.Popen(["modprobe", "uinput"]).wait()
        uinput_device_keyboard = uinput.Device(_CROS_KEYS_ALL)
        time.sleep(STARTUP_DELAY)
    return uinput_device_keyboard


def _uinput_translate_name(event_name):
    """
    Translates string |event_name| to uinput event.
    """
    return getattr(uinput.ev, "KEY_" + event_name.upper())


def _uinput_emit_keycombo(device, events, syn=True):
    """
    Wrapper for uinput.emit_combo. Emits sequence of events.
    Example: [KEY_LEFTCTRL, KEY_LEFTALT, KEY_F5]
    """
    time.sleep(DEFAULT_DELAY)
    device.emit_combo(events, syn)


def press_keys(keys):
    """Presses the given keys as one combination.
    @param key: A simple list of key strings, e.g. ['LEFTCTRL', 'F4']
    """
    events = [_uinput_translate_name(en) for en in keys]
    _uinput_emit_keycombo(_get_uinput_device_keyboard(), events)


@ItemRegistry.alias(["type"])
@ItemRegistry.register_function
def write(text: str):
    """Translates ASCII text to keystrokes and sends them as events.
    You can use '\n' for sending enter.
    @param text: string as a unique string text to send as events to keyboard, e.g. write('Hello World!')
    """
    events = _chars_to_events(text)
    device = _get_uinput_device_keyboard()
    for keys in events:
        _uinput_emit_keycombo(device, keys)


if __name__ == "__main__":
    write("Hello World!\n")
    press_keys(["LEFTALT", "TAB"])
