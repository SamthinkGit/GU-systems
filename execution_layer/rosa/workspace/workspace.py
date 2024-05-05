"""
Workspace Management Module
===========================

This module provides the Workspace class for managing workspace directories, both persistent and temporary.
It includes capabilities for adding files, managing temporary files, and safely closing the workspace with cleanup.

.. code-block:: python

    from core.workspace.workspace import Workspace
    from pathlib import Path

    # Use the workspace in a context manager
    with Workspace(name="my_workspace", temporal=True) as ws:

        temp_file_path = ws.add_temp_file('example.txt')
        print(f'Temporary file added at: {temp_file_path}')

        with open(temp_file_path, 'w') as f:
            f.write("Hello Workspace!")

        input("Press Enter to close workspace")

"""
import os
import uuid
from pathlib import Path
from shutil import rmtree

from execution_layer.rosa.constants import TEMP_FILE_PREFIX
from execution_layer.rosa.shared import get_rosa_path


class Workspace():
    """
    Manages workspace directories, supporting both persistent and temporary workspaces.

    :param path: The file path to the workspace directory. Defaults to a standard path if not provided.
    :param name: The name of the workspace. A unique name is generated for temporary workspaces if not provided.
    :param temporal: Indicates if the workspace is temporary (removed at closing). Defaults to False.

    :note: A temporal workspace will start with TEMP_FILE_PREFIX prefix

    :type path: Path, optional
    :type name: str, optional
    :type temporal: bool, optional
    """

    def __init__(self,
                 path: Path = None,
                 name: str = None,
                 temporal: bool = False) -> None:

        if path is not None:
            assert isinstance(path, Path), (
                "path passed to workspace is not a <Path> type"
            )

        self.is_temporal = temporal
        self.path = path or get_rosa_path() / 'workspace' / 'current_workspace'
        self.name = name

        # A temporal name for the directory is given if that name is not
        # specified so the 'current_workspace' dir doesn't get deleted as
        # a "temporal_directory"
        if self.is_temporal and self.name is None:
            self.name = f"ws_{uuid.uuid4()}"

        # If self.name is None, the user would be using 'current_directory' by
        # default
        if self.name is not None:

            self.name = self.name if not temporal else TEMP_FILE_PREFIX + self.name

            self.path = self.path / self.name

        if not self.path.exists():
            os.mkdir(self.path)

    def add_file(self, filename: str = None) -> Path:
        """
        Adds a new file to the workspace.

        :param filename: The name of the file to add. A unique name is generated if not provided.
        :type filename: str, optional
        :return: The path to the newly added file.
        :rtype: Path
        """

        filename = filename or f"{uuid.uuid4()}.txt"
        filepath = self.path / filename
        open(filepath, 'w').close()
        return filepath

    def add_temp_file(self, filename: str = None) -> Path:
        """
        Adds a new temporary file to the workspace.
        :note: All temporal files will be removed when the workspace is closed

        :param filename: The name of the temporary file to add. A unique name is generated if not provided.
        :type filename: str, optional
        :return: The path to the newly added temporary file.
        :rtype: Path
        """

        filename = filename or f"{uuid.uuid4()}.txt"
        filepath = self.path / str(TEMP_FILE_PREFIX + filename)
        open(filepath, mode='w').close()
        return filepath

    def close_workspace(self):
        """
        Closes the workspace. Deletes the workspace if it is temporary, otherwise only removes
        temporary files within.
        """

        if self.is_temporal \
           and not str(self.path).endswith("current_workspace"):

            rmtree(self.path)
            return

        file_list = [self.path / file for file in os.listdir(path=self.path)]

        for filename in file_list:
            if str(filename.name).startswith(TEMP_FILE_PREFIX):
                os.remove(filename)

    def __enter__(self):
        return self

    def __exit__(self, exc_type,  exc_value, traceback):
        if exc_type:
            print(f"An exception {exc_type} occurred with value {exc_value} \
                  in the current workspace. Traceback: {traceback}")
        self.close_workspace()
