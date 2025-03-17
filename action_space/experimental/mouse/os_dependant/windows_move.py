import pyautogui
pyautogui.FAILSAFE = False


def move(x, y):
    """
    Moves the cursor to a given absolute position.
    :param x: Horizontal coord (from 0 to MAX)
    :param y: Vertical coord (from 0 to MAX)
    """
    pyautogui.moveTo(x, y)


def click():
    pyautogui.click()


if __name__ == "__main__":
    while True:
        x = int(input("x: "))
        y = int(input("y: "))
        move(x, y)
