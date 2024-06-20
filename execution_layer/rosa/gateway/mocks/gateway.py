"""
Gateway Module (Mock)
===========================

This file contains functionalities for simulating functions to interact with the machine (Adding
some files, writing text...)
It is used to emulate connection with the gateway and is designed to be called mostly by higher
layers in syscore.

:note: Note that functions are registered with @ItemRegistry.register_function so the AI can call

them by using only the function id (More Information at :class:`gusystos.tools.registry.ItemRegistry`)
"""
import time

import pyautogui

from ecm.tools.registry import ItemRegistry
from execution_layer.rosa.workspace.workspace import Workspace


def assert_valid_text(text: str) -> None:
    """
    [Refactoring] Simple text sanitization
    """
    assert len(text) > 0, "Trying to write text that has length 0"


class IOMock():
    """
    Provides a mock interface for emulating I/O input to the machine.
    """

    @staticmethod
    @ItemRegistry.register_function
    def write(text: str) -> None:
        """MOCK"""
        assert_valid_text(text)
        pyautogui.write(text)


class OneFileWorkspaceMock(Workspace):
    """
    Simulates a workspace with a single file for writing. Easy interaction,
    can be used with <temporal> keyword for autoremoving changes in workspace
    """

    def __init__(self, temporal: bool = False) -> None:

        self.workspace = super().__init__(
            name="ws_" + str(time.strftime("%Y%m%d_%H%M%S")),
            temporal=temporal
            )

        self.file = self.add_file(
            filename=str(time.strftime("%Y%m%d_%H%M%S")) + ".txt"
        )

    def write(self, text: str):
        with open(self.file, mode='a') as f:
            f.write(text)

    @staticmethod
    @ItemRegistry.register_function
    def write_with(workspace, text: str):
        """MOCK"""
        workspace.write(text)


if __name__ == '__main__':

    with OneFileWorkspaceMock(temporal=True) as ws:

        ws.write("Hello World!\n")
        ws.write("Hello World!\n")

        input(f"Succesfully written to {ws.file}\nTap ENTER to clean workspace")

    print("Workspace cleaned, exit")
