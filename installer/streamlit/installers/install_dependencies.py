from typing import Literal

import streamlit as st
from config import get_repository_root_path
from installers.description import InstallerDescription
from installers.persistent_shell import PersistentShell


def check_dependencies(
    pip_list: str,
    type: Literal["cognition", "execution", "ecm"],
    module: str,
) -> tuple[bool, list[str]]:

    directory_name = None
    match type:
        case "cognition":
            directory_name = "cognition_layer"
        case "execution":
            directory_name = "execution_layer"
        case "ecm":
            directory_name = "ecm"
        case _:
            raise ValueError(f"Invalid type: {type}")
    path = (
        get_repository_root_path()
        / "installer"
        / "streamlit"
        / "dependencies"
        / directory_name
        / f"requirements_{module.lower()}.txt"
    )
    dependencies = []
    for line in path.read_text().splitlines():
        if line.startswith("#"):
            continue
        dep = line.split("==")[0]
        dependencies.append(dep)

    dependencies = [dep for dep in dependencies if dep.strip() != ""]

    installed_dependencies = []
    for line in pip_list.splitlines():
        words = line.split()
        if len(words) == 2:
            installed_dependencies.append(line.split()[0].lower())

    not_installed = []
    for dependency in dependencies:
        if get_fixed_package_name(dependency.lower()) not in installed_dependencies:
            not_installed.append(dependency)

    if not_installed:
        return False, not_installed
    return True, []


def install_dependencies(
    shell: PersistentShell, description: InstallerDescription
) -> bool:

    shell.send_command(f"cd '{get_repository_root_path().resolve().absolute()}'")

    exit_code, _ = shell.read_output()
    if exit_code != 0:
        st.error(
            "Error changing directory to the root path. Please check your Python installation."
        )
        return False
    if (
        not _install_cognition_layer_dependencies(shell, description)
        or not _install_execution_layer_dependencies(shell, description)
        or not _install_ecm_dependencies(shell, description)
    ):
        return False
    return True


def _install_cognition_layer_dependencies(
    shell: PersistentShell, description: InstallerDescription
) -> bool:
    """
    Install the dependencies for the cognition layer.
    """

    cognition_path = (
        get_repository_root_path()
        / "installer"
        / "streamlit"
        / "dependencies"
        / "cognition_layer"
    )

    for layer in description.cognition_layers + ["base"]:
        requirements_path = f"{cognition_path}/requirements_{layer.lower()}.txt"
        success = pip_install(shell, requirements_path, description)
        if not success:
            st.error(
                f"Error installing the dependencies for the cognition layer {layer}."
                " Please check the requirements file."
            )
            return False
    return True


def _install_execution_layer_dependencies(
    shell: PersistentShell, description: InstallerDescription
) -> bool:
    """
    Install the dependencies for the cognition layer.
    """

    execution_path = (
        get_repository_root_path()
        / "installer"
        / "streamlit"
        / "dependencies"
        / "execution_layer"
    )

    for layer in description.execution_layers:
        requirements_path = f"{execution_path}/requirements_{layer.lower()}.txt"
        success = pip_install(shell, requirements_path, description)
        if not success:
            st.error(
                f"Error installing the dependencies for the execution layer {layer}."
                " Please check the requirements file."
            )
            return False

    return True


def _install_ecm_dependencies(
    shell: PersistentShell, description: InstallerDescription
) -> bool:
    """
    Install the dependencies for the cognition layer.
    """

    ecm_path = (
        get_repository_root_path() / "installer" / "streamlit" / "dependencies" / "ecm"
    )

    for layer in description.ecm_dependencies:
        requirements_path = f"{ecm_path}/requirements_{layer.lower()}.txt"
        success = pip_install(shell, requirements_path, description)
        if not success:
            st.error(
                f"Error installing the dependencies for the execution layer {layer}."
                " Please check the requirements file."
            )
            return False

    return True


def get_fixed_package_name(package_name: str) -> str:
    match package_name:
        case "onnxruntime-tools":
            return "onnxruntime"
        case "langchain_openai":
            return "langchain-openai"
        case "langchain_ollama":
            return "langchain-ollama"
    if package_name.startswith("./external/"):
        return package_name.split("/")[-1]
    return package_name


def get_pip_list(shell: PersistentShell) -> list[str]:
    """
    Get the list of installed packages.
    """
    shell.send_command("python -m pip list")
    return shell.read_output()


def pip_install(
    shell: PersistentShell, requirements: str, description: InstallerDescription
):
    pip = "pip"
    flags = ""
    if description.install_with_conda:

        if shell.is_windows:
            env_name = "$env:CONDA_PREFIX"
        else:
            env_name = "$CONDA_PREFIX"

        shell.send_command(f"echo {env_name}")
        exit_code, conda_prefix = shell.read_output()
        if shell.is_windows:
            conda_prefix = conda_prefix.split("\n")[-4].strip()
        else:
            conda_prefix = conda_prefix.split("\n")[-3].strip()

        if shell.is_windows:
            pip = f"{conda_prefix}\\Scripts\\pip.exe"
        else:
            pip = f"{conda_prefix}/bin/pip"
        flags = "--ignore-installed"

    shell.send_command(f"{pip} install {flags} -r {requirements}")
    exit_code, _ = shell.read_output()
    if exit_code != 0:
        return False
    return True
