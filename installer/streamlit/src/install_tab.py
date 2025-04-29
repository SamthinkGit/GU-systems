import streamlit as st
from installers.conda import activate_conda_environment
from installers.conda import check_conda_installation
from installers.conda import create_conda_environment
from installers.conda import get_conda_envs
from installers.conda import get_current_conda_env
from installers.description import InstallerDescription
from installers.persistent_shell import PersistentShell

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


def load_install_tab():
    st.subheader("Installation in progress...")
    st.write(
        "You can debug the output of the installation in the shell running streamlit."
    )
    description: InstallerDescription = st.session_state.installation_description
    container = st.container(height=600, border=True)

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

        expander = st.expander(label="**Checking conda environment**", expanded=True)
        log_starting_tab("Checking conda environment")
        with expander:
            success = load_conda_env(shell, description)
            if not success:
                return
            if success and description.install_with_conda:
                st.success("Conda environment loaded successfully.")
            if success and not description.install_with_conda:
                st.success(
                    "No conda environment required. Proceeding with installation."
                )


def log_starting_tab(tab_name: str):
    print("=" * 20 + f" {tab_name} " + "=" * 20)


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
