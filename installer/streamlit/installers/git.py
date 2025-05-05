from config import get_repository_root_path
from installers.persistent_shell import PersistentShell


def git_update(shell: PersistentShell) -> bool:
    shell.send_command(f"cd {get_repository_root_path().resolve().absolute()}")
    exit_code, output = shell.read_output()
    if exit_code != 0:
        return False

    shell.send_command("git pull")
    exit_code, output = shell.read_output()
    if exit_code != 0:
        return False

    shell.send_command("git submodule update --init --recursive")
    exit_code, output = shell.read_output()
    if exit_code != 0:
        return False

    shell.send_command("git submodule foreach git pull")
    exit_code, output = shell.read_output()
    return exit_code == 0
