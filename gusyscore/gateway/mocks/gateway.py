import time

import pyautogui

from gusyscore.ros2.tools.registry import ItemRegistry
from gusyscore.workspace.workspace import Workspace


def assert_valid_text(text: str) -> None:
    assert len(text) > 0, "Trying to write text that has length 0"


class IOMock():

    @staticmethod
    @ItemRegistry.register_function
    def write(text: str) -> None:
        assert_valid_text(text)
        pyautogui.write(text)


class OneFileWorkspaceMock(Workspace):

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
        workspace.write(text)


if __name__ == '__main__':

    with OneFileWorkspaceMock(temporal=True) as ws:

        ws.write("Hello World!\n")
        ws.write("Hello World!\n")

        input(f"Succesfully written to {ws.file}\nTap ENTER to clean workspace")

    print("Workspace cleaned, exit")
