import streamlit as st
from installers.description import run_command_live
from installers.persistent_shell import PersistentShell


def get_conda_envs() -> list[str]:
    exit_code, result = run_command_live(
        ["conda", "env", "list"], write_output_with_st=False
    )
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


def check_conda_installation(write_output_with_st: bool = True) -> bool:
    """
    Check if conda is installed.
    """
    success, _ = run_command_live(
        ["conda", "--version"], write_output_with_st=write_output_with_st
    )
    return success == 0


def create_conda_environment(
    env_name: str, python_version: str = "3.10", write_output_with_st: bool = True
) -> bool:
    """
    Create a conda environment.
    """
    success, _ = run_command_live(
        ["conda", "create", "-n", env_name, "--yes", f"python={python_version}"],
        write_output_with_st=write_output_with_st,
    )
    return success == 0


def get_current_conda_env(
    shell: PersistentShell, write_output_with_st: bool = False
) -> str:
    """
    Get the name of the currently active conda environment.
    """
    shell.send_command("conda info")
    success, conda_info = shell.read_output(write_output_with_st=False)

    if write_output_with_st:
        st.markdown(f"```bash\n{conda_info}```")

    if success != 0:
        raise RuntimeError(f"Error getting conda info.\n{conda_info}")

    for line in conda_info.splitlines():
        if line.strip().startswith("active environment"):
            return line.split(":")[1].strip()

    return ""


def activate_conda_environment(
    shell: PersistentShell, env_name: str, write_output_with_st: bool = True
) -> bool:
    """Activate a conda environment."""
    shell.send_command("conda info")
    success, conda_info = shell.read_output(write_output_with_st)
    if success != 0:
        return False

    if f"active environment : {env_name}" in conda_info:
        st.write(f"Conda environment {env_name} is already activated.")
        return True

    shell.send_command(f"conda activate {env_name}")
    success, _ = shell.read_output(write_output_with_st)
    return success == 0
