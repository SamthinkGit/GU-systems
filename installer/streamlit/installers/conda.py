import streamlit as st
from installers.description import run_command_live


def check_conda_installation(write_output_with_st: bool = True) -> bool:
    """
    Check if conda is installed.
    """
    return run_command_live(
        ["conda", "--version"], write_output_with_st=write_output_with_st
    )


def create_conda_environment(
    env_name: str, python_version: str = "3.10", write_output_with_st: bool = True
) -> bool:
    """
    Create a conda environment.
    """
    return run_command_live(
        ["conda", "create", "-n", env_name, "--yes", f"python={python_version}"],
        write_output_with_st=write_output_with_st,
    )


def activate_conda_environment(
    env_name: str, write_output_with_st: bool = True
) -> bool:
    """Activate a conda environment."""
    conda_info = run_command_live(["conda", "info"], write_output_with_st=False)

    if f"active environment : {env_name}" in conda_info:
        st.write(f"Conda environment {env_name} is already activated.")
        return True

    return run_command_live(["conda", "activate", env_name], write_output_with_st)
