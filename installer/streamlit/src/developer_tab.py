import streamlit as st
from installers.conda import activate_conda_environment
from installers.conda import check_conda_installation
from installers.conda import get_conda_envs
from installers.conda import get_current_conda_env
from installers.persistent_shell import PersistentShell


def init():
    frame = st.empty()
    frame.markdown("# `Connecting to internal shell...`")

    shell = PersistentShell()
    st.session_state.shell = shell
    st.session_state.conda_init = True
    installed = check_conda_installation(write_output_with_st=False)
    st.session_state.conda_installed = installed

    if installed:
        st.session_state.conda_envs = get_conda_envs()
        st.session_state.current_conda_env = get_current_conda_env(
            shell, write_output_with_st=False
        )
    else:
        st.session_state.conda_envs = []
        st.session_state.current_conda_env = "None"

    frame.empty()


def reload_conda():
    st.session_state.shell.close()
    st.session_state.shell = PersistentShell()
    st.session_state.conda_init = False
    init()


def set_conda_env():
    env = st.session_state.conda_env_select
    activate_conda_environment(st.session_state.shell, env, write_output_with_st=False)
    st.session_state.current_conda_env = get_current_conda_env(
        st.session_state.shell, write_output_with_st=False
    )


def load_conda_selection_section():
    c1, *_, c2 = st.columns(4)
    with c1:
        st.subheader("Conda Environment")
    with c2:
        st.button(
            "Reload",
            help="Reload all conda variables in the app.",
            on_click=reload_conda,
            use_container_width=True,
        )

    if "conda_init" not in st.session_state or not st.session_state.conda_init:
        st.warning("conda_init has not been initialized. Please restart the app.")
        return

    conda_available = st.session_state.conda_installed

    if not conda_available:
        st.info(
            """
            **Conda does not appear installed.**\n
            Please install conda to use this feature.
            """
        )

    c1, c2 = st.columns(2)
    with c1:
        st.write("**Change conda environment:**")

    with c2:
        envs = ["None"] + st.session_state.conda_envs
        st.selectbox(
            "",
            options=envs,
            label_visibility="collapsed",
            help="Select the conda environment you want to use.",
            disabled=not conda_available,
            key="conda_env_select",
            on_change=set_conda_env,
            index=envs.index(st.session_state.current_conda_env)
        )

    c1, c2 = st.columns(2)
    with c1:
        st.markdown("**Current active conda environment**")
    with c2:
        st.button(
            st.session_state.current_conda_env, disabled=True, use_container_width=True
        )

    st.divider()


def load_debug_tab():
    st.subheader("Conda")
    conda_init_output = st.button("Check conda installation logs")
    if conda_init_output:
        container = st.container(border=True)
        with container:
            st.markdown("Running: `conda --version`")
            success = check_conda_installation(write_output_with_st=True)
            st.write("Success:", success)

    conda_init_output = st.button("Check current conda env")
    if conda_init_output:
        container = st.container(border=True)
        with container:
            st.markdown("Running: `conda info`")
            env = get_current_conda_env(
                st.session_state.shell, write_output_with_st=True
            )
            st.write("Parsed env:", env)


def load_config_tab():
    load_conda_selection_section()


def load_developer_tab():
    if "conda_init" not in st.session_state:
        init()

    st.header("Developer Installation")
    config, debug = st.tabs(["Selection", "Debug"])
    with config:
        load_config_tab()
    with debug:
        load_debug_tab()
