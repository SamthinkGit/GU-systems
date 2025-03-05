"""
Calibrated Move
==============================
This file uses the .env configuration to find a translation between uinput mouse
traveled distance and the real movement shown in the display. With this
equilibrium you can move the mouse throught the display to a given coordinates.

Note: Pyautogui or other libraries that use absolute coord can be used in
X11 based distros, since Ubuntu-22 Wayland window management, this libraries
does not work when using different apps. We are working on finding a better
approach to this.

Note: This .env file has been teste on a 1920x1080 display.

Insights:
This algorithm works by shifting the mouse to the upper left corner, sending
a high ammmount of move events. Then it keeps the count of the traveled distance
using reactive navigation.
"""
import os
import sys
import time

import uinput

CALIBRATED_MOUSE_DEVICE = uinput.Device(
    [
        uinput.BTN_LEFT,
        uinput.BTN_RIGHT,
        uinput.REL_X,
        uinput.REL_Y,
    ]
)

UINPUT_START_TIME = time.perf_counter()

X_SENSIBILITY = float(os.getenv("CURSOR_SENSIBILITY_X"))
Y_SENSIBILITY = float(os.getenv("CURSOR_SENSIBILITY_Y"))
MOUSE_ACCURACY = 3
MOUSE_STEP = int(os.getenv("MOUSE_STEP"))


def move(x, y):
    """
    Moves the cursor to a given absolute position.
    :param x: Horizontal coord (from 0 to MAX)
    :param y: Vertical coord (from 0 to MAX)
    """
    global CALIBRATED_MOUSE_DEVICE
    global UINPUT_START_TIME

    if time.perf_counter() - UINPUT_START_TIME < 1:
        time.sleep(1)

    CALIBRATED_MOUSE_DEVICE.emit(uinput.REL_X, -10000)
    CALIBRATED_MOUSE_DEVICE.emit(uinput.REL_Y, -10000)

    pos_x = 0
    pos_y = 0

    distance = MOUSE_ACCURACY + 1
    while distance > MOUSE_ACCURACY:

        direction_x = -1 if pos_x > x else 1
        direction_y = -1 if pos_y > y else 1

        CALIBRATED_MOUSE_DEVICE.emit(uinput.REL_X, direction_x * MOUSE_STEP)
        CALIBRATED_MOUSE_DEVICE.emit(uinput.REL_Y, direction_y * MOUSE_STEP)

        pos_x += X_SENSIBILITY * direction_x
        pos_y += Y_SENSIBILITY * direction_y
        distance = abs(x - pos_x) + abs(y - pos_y)
        time.sleep(0.0005)


def click():
    CALIBRATED_MOUSE_DEVICE.emit(uinput.BTN_LEFT, 1)
    CALIBRATED_MOUSE_DEVICE.emit(uinput.BTN_LEFT, 0)


if __name__ == "__main__":
    move(int(sys.argv[1]), int(sys.argv[2]))
