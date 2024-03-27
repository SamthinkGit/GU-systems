import pyautogui

from pathlib import Path
from gusyscore.workspace.workspace import Workspace


def assert_valid_text(text: str, func: callable) -> None:
    assert len(text) > 0, (
        f"Trying to write text in {func.__name__} that has length 0"
    )


def write(text: str) -> None:
    assert_valid_text(text, func=write)
    pyautogui.write(text)


def write_to_workspace(
        text: str,
        workspace: Workspace = Workspace(),
        temp_file: bool = False,
        filename: str = None) -> Path:

    assert_valid_text(text, func=write_to_workspace)
    if temp_file:
        file = workspace.add_temp_file(filename=filename)
    else:
        file = workspace.add_file(filename=filename)

    with open(file, 'w') as f:
        f.write(text)

    return file


if __name__ == '__main__':

    with Workspace(temporal=True, name="My_Workspace") as ws:

        file = write_to_workspace("Hello World!\n", workspace=ws)

        with open(file, mode='a') as f:
            f.write("Hello World!\n")
            f.write("Hello World!\n")

        input(f"""A new temporal file has been built in {file}
              Use Enter to close the workspace""")

    print("Workspace cleaned, exit")
