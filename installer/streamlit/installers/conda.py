from installers.description import run_command_live
from installers.persistent_shell import PersistentShell


def get_conda_envs() -> list[str]:
    exit_code, result = run_command_live(["conda", "env", "list"])
    if exit_code != 0:
        raise RuntimeError("Error getting conda environments.")

    envs = []
    for line in result.splitlines():
        if line.startswith("#"):
            continue
        splitted_line = line.split()
        if len(splitted_line) > 1:
            envs.append(splitted_line[0])
    return envs


def check_conda_installation() -> bool:
    """
    Check if conda is installed.
    """
    success, _ = run_command_live(["conda", "--version"])
    return success == 0


def create_conda_environment(
    shell: PersistentShell,
    env_name: str,
    python_version: str = "3.10",
) -> tuple[bool, str]:
    """
    Create a conda environment.
    """
    shell.send_command(f"conda create -n {env_name} --yes python={python_version}")
    success, output = shell.read_output()
    return success == 0, output


def get_current_conda_env(shell: PersistentShell) -> tuple[str, str]:
    """
    Get the name of the currently active conda environment.
    """
    shell.send_command("conda info")
    success, output = shell.read_output()

    if success != 0:
        raise RuntimeError(f"Error getting conda info.\n{output}")

    for line in output.splitlines():
        if line.strip().startswith("active environment"):
            return line.split(":")[1].strip()

    return "", output


def activate_conda_environment(
    shell: PersistentShell, env_name: str
) -> tuple[bool, str]:
    """Activate a conda environment."""
    if not shell.is_windows:
        shell.send_command('eval "$(conda shell.bash hook)"')
        success, output = shell.read_output()
        if success != 0:
            return False, output

    shell.send_command("conda init --all")
    success, output = shell.read_output()
    if success != 0:
        return False, output

    shell.send_command("conda info")
    success, output = shell.read_output()
    if success != 0:
        return False, output

    if f"active environment : {env_name}" in output:
        return True, output

    shell.send_command(f"conda activate {env_name}")
    success, output = shell.read_output()
    return success == 0, output
