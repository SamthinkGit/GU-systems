import time

import streamlit as st
from config import get_repository_root_path
from installers.api_keys import add_to_dotenv
from installers.conda import activate_conda_environment
from installers.conda import check_conda_installation
from installers.conda import create_conda_environment
from installers.conda import get_conda_envs
from installers.conda import get_current_conda_env
from installers.description import InstallerDescription
from installers.fix_execution_policy_windows import fix_execution_policy_permanent
from installers.git import git_update
from installers.install_dependencies import install_dependencies
from installers.persistent_shell import PersistentShell
from installers.precommit import configure_precommit
from installers.precommit import install_precommit
from installers.pythonpath import add_to_pythonpath

MAX_CONDA_RETRIES = 5
PYTHON_VERSION = "3.10"


def load_confirm_install_tab():
    if "installation_description" not in st.session_state:
        st.warning(
            "Please confirm the installation options from the sidebar to proceed."
        )
        return
    st.subheader("Confirm the installation options:")
    description: InstallerDescription = st.session_state.installation_description
    st.write(description)
    confirm = st.button("Proceed with installation", use_container_width=True)
    if confirm:
        st.session_state.page = "_install"
    st.rerun()


def log_starting_tab(tab_name: str):
    print("=" * 20 + f" {tab_name} " + "=" * 20)


def load_install_tab():
    st.subheader("Installation in progress...")
    st.write(
        "You can debug the output of the installation in the shell running streamlit."
    )
    description: InstallerDescription = st.session_state.installation_description
    container = st.container(height=500, border=True)

    def stop_shell_function():
        if "shell" in st.session_state:
            shell: PersistentShell = st.session_state.shell
            shell.close()
            st.success("Shell stopped successfully.")
            st.session_state.stop = True
        else:
            st.warning("No shell to stop.")

    stop_button = st.button(  # noqa
        "Stop installation",
        use_container_width=True,
        on_click=stop_shell_function,
    )
    if stop_button:
        return

    with container:
        expander = st.expander(label="**Initializing a new Shell**", expanded=True)
        with expander:
            shell = PersistentShell()
            st.session_state.shell = shell
            st.success("Shell initialized successfully.")

        if description.git_pull:
            log_starting_tab("Updating repository")
            with st.expander(label="**Updating repository**", expanded=True):
                success = git_update(shell)
                if not success:
                    st.error(
                        "Failed to update the repository. Please check the logs in the shell."
                    )
                    return
                st.success("Repository updated successfully.")

        expander = st.expander(label="**Checking conda environment.**", expanded=True)
        log_starting_tab("Checking conda environment")
        with expander:
            with st.spinner("Loading conda environment..."):
                success = load_conda_env(shell, description)
            if not success:
                st.error(
                    "Failed to load conda environment. Please check the installation."
                )
                return
            if success and description.install_with_conda:
                st.success("Conda environment loaded successfully.")
            if success and not description.install_with_conda:
                st.success(
                    "No conda environment required. Proceeding with installation."
                )
        expander = st.expander("**Installing Python dependencies**", expanded=True)
        with expander:
            log_starting_tab("Installing Python dependencies")
            with st.spinner("Installing...", show_time=True):
                success = install_dependencies(shell, description=description)
            if not success:
                st.error(
                    "Failed to install Python dependencies. Please check the logs in the shell."
                )
                return
            st.success("All python dependencies installed successfully.")

        if description.os.lower == "windows":
            expander = st.expander("**Fixing windows policy**", expanded=True)
            log_starting_tab("Fixing windows policy")
            with expander:
                try:
                    success = fix_execution_policy_permanent()
                except PermissionError:
                    st.error(
                        "Administrator privileges required to change Execution Policy permanently."
                        "\nPlease re-run the installer as Administrator."
                    )
                if not success:
                    st.error(
                        "Failed to fix the execution policy. Please check the logs in the shell."
                    )
                    return
                st.success("Windows execution policy fixed successfully.")

        if description.precommit:
            expander = st.expander("**Installing pre-commit**", expanded=True)
            log_starting_tab("Installing pre-commit")
            with expander:
                with st.spinner("Installing...", show_time=True):
                    success = install_precommit(shell)
                if not success:
                    st.error(
                        "Failed to install pre-commit. Please check the logs in the shell."
                    )
                    return
                with st.spinner("Configuring Precommit...", show_time=True):
                    success = configure_precommit(shell)
                if not success:
                    st.error(
                        "Failed to configure pre-commit. Please check the logs in the shell."
                    )
                    return
                st.success("Pre-commit installed and configured successfully.")

        if len(description.api_keys) > 0:
            expander = st.expander("**Setting up API keys**", expanded=True)
            with expander:
                for key, val in description.api_keys.items():
                    add_to_dotenv(
                        key, val, dotenv_path=get_repository_root_path() / ".env"
                    )

        if description.setup_python_path:
            expander = st.expander("**Setting up Python path**", expanded=True)
            with expander:
                os = "Windows" if description.os == "windows" else "LinuxLike"
                add_to_pythonpath(get_repository_root_path(), os)
                st.success("Python path added successfully.")
        time.sleep(1)
        st.session_state.page = "_installation_success"
        st.rerun()


def load_installation_success_tab():
    description: InstallerDescription = st.session_state.installation_description
    st.subheader("Installation completed successfully!")
    st.write(
        "You can now start using the installed packages and tools. "
        "If you encounter any issues, please check the logs in the shell."
    )
    st.markdown("Use the following command to start the ECM:")
    st.code(
        f"""
    cd '{get_repository_root_path().resolve().absolute()}'
    conda activate {description.conda_path} # only if using conda
    python ecm/core/main/main.py --host --agent {description.cognition_layers[0]}
            """,
        language="bash",
    )
    st.balloons()
    st.write("You can now close the shell to exit the installer.")


def load_conda_env(shell: PersistentShell, description: InstallerDescription):
    installed = check_conda_installation()
    if not installed and (
        not description.install_with_conda or description.conda_path == "None"
    ):
        return True
    if not installed and description.install_with_conda:
        st.error("Conda is not installed. Please install conda and try again.")
        return False

    current_env = get_current_conda_env(shell)
    st.markdown(f"**current conda environment**: `{current_env}`")
    retries = 0

    if description.conda_path == "None":
        while current_env.lower() != "none":
            st.markdown(f"**current conda environment**: `{current_env}`")
            shell.send_command("conda deactivate")
            success, output = shell.read_output()
            if not success:
                st.error("Failed to deactivate conda environment.")
                return False

            current_env = get_current_conda_env(shell)
            retries += 1
            if retries >= MAX_CONDA_RETRIES:
                st.error(
                    "Failed to deactivate conda environment after multiple attempts."
                )
                return False

        return True
    else:
        available_envs = get_conda_envs()
        st.markdown(f"Available conda environments: `{available_envs}`")

        if description.conda_path not in available_envs:
            st.write(
                f"Conda environment `{description.conda_path}` not found. Creating a new conda environment."
            )
            success, output = create_conda_environment(
                shell,
                description.conda_path,
                python_version=PYTHON_VERSION,
            )
            if not success:
                st.error(
                    f"Failed to create conda environment `{description.conda_path}`. "
                    "Please check the conda installation."
                )
                return False

        st.write(f"Activating conda environment `{description.conda_path}`")
        success, output = activate_conda_environment(shell, description.conda_path)
        current_env = get_current_conda_env(shell)
        if current_env.lower() != description.conda_path.lower():
            st.error(
                f"Failed to activate conda environment {description.conda_path}. Please check the environment name."
            )
            return False
        return True
