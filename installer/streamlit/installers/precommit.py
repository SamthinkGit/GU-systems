from config import get_repository_root_path
from installers.persistent_shell import PersistentShell


def install_precommit(shell: PersistentShell):
    shell.send_command("pip install pre-commit")
    exit_code, output = shell.read_output()
    return exit_code == 0


def configure_precommit(shell: PersistentShell):
    shell.send_command(f"cd '{get_repository_root_path().resolve().absolute()}'")
    exit_code, output = shell.read_output()
    if exit_code != 0:
        return False
    shell.send_command("pre-commit install")
    exit_code, output = shell.read_output()
    return exit_code == 0
