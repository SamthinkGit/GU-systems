"""
KeyNames
==============================
This class contains only a dictionary for multiple keys that can be
used by an AI.
"""


class KeyNames:

    CTRL: list[str] = [
        "ctrl",
        "control",
        "ctrlkey",
        "controlkey",
        "ctrl_key",
        "control_key",
        "key_ctrl",
        "key_control",
        "ctrl_pressed",
        "ctrl_pressed",
        "control_pressed",
        "ctrl_modifier",
        "control_modifier",
        "ctrl_button",
        "control_button",
    ]
    SHIFT: list[str] = [
        "shift",
        "shiftkey",
        "shift_key",
        "key_shift",
        "shift_pressed",
        "shift_modifier",
        "shift_button",
        "shift_flag",
        "shift_status",
        "shift_state",
        "shift_active",
    ]
    ALT: list[str] = [
        "alt",
        "altkey",
        "alt_key",
        "key_alt",
        "alt_pressed",
        "is_alt_pressed",
        "alt_modifier",
        "alt_button",
        "alt_flag",
        "alt_status",
        "alt_state",
        "alt_active",
    ]
    META: list[str] = [
        "meta",
        "metakey",
        "meta_key",
        "key_meta",
        "meta_pressed",
        "meta_modifier",
        "meta_button",
        "meta_trigger",
    ]
    ESC: list[str] = [
        "esc",
        "escape",
        "escapekey",
        "escape_key",
        "key_escape",
        "escape_pressed",
        "escape_button",
        "escape_trigger",
    ]
    TAB: list[str] = [
        "space",
        "spacekey",
        "space_key",
        "key_space",
        "space_pressed",
        "space_button",
        "space_trigger",
    ]
    BACKSPACE: list[str] = [
        "back",
        "backkey",
        "backspace",
        "backspacekey",
        "backspace_key",
        "key_backspace",
        "backspace_pressed",
        "backspace_button",
        "backspace_trigger",
    ]
    DELETE: list[str] = [
        "del",
        "delete",
        "deletekey",
        "delete_key",
        "del_key",
        "delpressed",
        "delete_pressed",
        "deletepressed",
    ]
    HOME: list[str] = [
        "home",
        "home_key",
        "homekey",
        "homepressed",
        "is_home",
        "homestatus",
        "home_status",
    ]
    END: list[str] = [
        "end",
        "end_key",
        "endkey",
        "endkeypress",
        "press_end",
        "endpressed",
    ]
