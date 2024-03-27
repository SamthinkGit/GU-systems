import uuid
import os

from shutil import rmtree
from pathlib import Path
from gusyscore.core import get_core_path
from gusyscore.constants import TEMP_FILE_PREFIX

class Workspace():

    def __init__(self, path: Path = None, name: str=None, temporal: bool = False) -> None:

        if path is not None:
            assert isinstance(path, Path), "path passed to workspace is not a <Path> type"

        self.is_temporal = temporal
        self.path = path or get_core_path() / 'workspace' / 'current_workspace'

        if name is not None:
            name = name if not temporal else TEMP_FILE_PREFIX + name
            self.path = self.path / name

        if not self.path.exists():
            os.mkdir(self.path)
        
    def add_file(self, filename: str = None) -> Path:
        filename = filename or f"{uuid.uuid4()}.txt" 
        filepath = self.path / filename
        open(filepath, 'w').close()
        return filepath

    def add_temp_file(self, filename: str = None) -> Path:
        filename = filename or f"{uuid.uuid4()}.txt" 
        filepath = self.path / str(TEMP_FILE_PREFIX + filename)
        open(filepath, mode='w').close()
        return filepath

    def close_workspace(self):

        if self.is_temporal:
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
            print(f"An exception {exc_type} occurred with value {exc_value} in the current workspace. Traceback: {traceback}")
        self.close_workspace()


