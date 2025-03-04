from ecm.tools.get_platform import osmethod


@osmethod
def move(os, x: int, y: int):
    if os == "Windows":
        from action_space.experimental.mouse.os_dependant.windows_move import move

        move(x, y)
    else:
        from action_space.experimental.mouse.os_dependant.linux_calibrated_move import (
            move,
        )

        move(x, y)


@osmethod
def click(os):
    if os == "Windows":
        from action_space.experimental.mouse.os_dependant.windows_move import click

        click()
    else:
        from action_space.experimental.mouse.os_dependant.linux_calibrated_move import (
            click,
        )

        click()
