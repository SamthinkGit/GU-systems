import streamlit as st
from config import COGNITION_LAYER_OPTIONS
from config import ECM_OPTIONS
from config import EXECUTION_LAYER_OPTIONS
from config import LATEST_COGNITION_LAYER
from config import LATEST_EXECUTION_LAYER
from installers.conda import activate_conda_environment
from installers.conda import check_conda_installation
from installers.conda import get_conda_envs
from installers.conda import get_current_conda_env
from installers.install_dependencies import check_dependencies
from installers.install_dependencies import reload_pip_list_cache
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

    try:
        load_dependencies_summary()
    except IndexError:
        st.error("Error loading dependencies summary. Please refresh the app (F5).")
    frame.empty()


def reload_conda():
    st.session_state.shell.close()
    st.session_state.shell = PersistentShell()
    st.session_state.conda_init = False
    init()


def set_conda_env():
    env = st.session_state.conda_env_select
    if env.lower() == "none":
        shell: PersistentShell = st.session_state.shell
        shell.send_command("conda deactivate")
        st.session_state.current_conda_env = "None"
    else:
        activate_conda_environment(
            st.session_state.shell, env, write_output_with_st=False
        )
        st.session_state.current_conda_env = get_current_conda_env(
            st.session_state.shell, write_output_with_st=False
        )

    load_dependencies_summary()


def load_dependencies_summary():
    reload_pip_list_cache()
    shell = st.session_state.shell
    cognition_options = [val for val in COGNITION_LAYER_OPTIONS if val.lower() != "all"]
    execution_options = [val for val in EXECUTION_LAYER_OPTIONS if val.lower() != "all"]
    data = {
        type: {
            name: {
                "installed": check_dependencies(shell, type, name)[0],
                "missing": check_dependencies(shell, type, name)[1],
            }
            for name in options
        }
        for type, options in [
            ("cognition", cognition_options),
            ("execution", execution_options),
            ("ecm", ECM_OPTIONS),
        ]
    }
    st.session_state.dependencies_summary = data


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
            index=envs.index(st.session_state.current_conda_env),
        )

    c1, c2 = st.columns(2)
    with c1:
        st.markdown("**Current active conda environment**")
    with c2:
        st.button(
            st.session_state.current_conda_env, disabled=True, use_container_width=True
        )

    st.divider()

    c1, c2 = st.columns(2)
    with c1:
        st.markdown("**Install Everything!**")
    with c2:
        default = st.toggle("Add", key="install_all")

    st.divider()

    c1, *_, c2 = st.columns(4)
    st.subheader("Cognition Layer")
    cognition_options = [val for val in COGNITION_LAYER_OPTIONS if val.lower() != "all"]
    execution_options = [val for val in EXECUTION_LAYER_OPTIONS if val.lower() != "all"]
    st.markdown(f"Latest: `{LATEST_COGNITION_LAYER}`")
    for layer in cognition_options:
        c1, c2 = st.columns(2)
        with c1:
            st.write(f"**{layer}**")
        with c2:
            disabled = st.session_state.dependencies_summary["cognition"][layer][
                "installed"
            ]
            label = ":blue[Installed]" if disabled else "Install"
            st.toggle(
                label,
                disabled=disabled,
                key=f"cognition_{layer}",
                value=not disabled and default,
            )

    st.markdown("> Base cognition layer is needed for all cognition layers.")
    st.subheader("Execution Layer")
    st.markdown(f"Latest: `{LATEST_EXECUTION_LAYER}`")
    for layer in execution_options:
        c1, c2 = st.columns(2)
        with c1:
            st.write(f"**{layer}**")
        with c2:
            disabled = st.session_state.dependencies_summary["execution"][layer][
                "installed"
            ]
            label = ":blue[Installed]" if disabled else "Install"
            st.toggle(
                label,
                disabled=disabled,
                key=f"execution_{layer}",
                value=not disabled and default,
            )

    st.subheader("Core Components")
    for component in ECM_OPTIONS:
        c1, c2 = st.columns(2)
        with c1:
            st.write(f"**{component}**")
        with c2:
            disabled = st.session_state.dependencies_summary["ecm"][component][
                "installed"
            ]
            label = ":blue[Installed]" if disabled else "Install"
            st.toggle(
                label,
                disabled=disabled,
                key=f"ecm_{component}",
                value=not disabled and default,
            )

    st.divider()
    st.subheader("Client/Host Templates")
    st.markdown(
        "> This mode is still not implemented. Future versions will have this feature."
    )

    st.divider()
    st.subheader("Environment")
    c1, c2 = st.columns(2)
    with c1:
        st.markdown("**Add repository to PYTHONPATH**")
    with c2:
        st.toggle("Add", disabled=False, key="pythonpath", value=default)

    c1, c2 = st.columns(2)
    with c1:
        st.markdown("**Install Precommit tools (Needed for pushing to github)**")
    with c2:
        st.toggle("Add", disabled=False, key="precommit", value=default)

    st.divider()
    c1, c2 = st.columns(2)
    with c1:
        st.markdown("**Add API keys to dotenv**")
    with c2:
        st.toggle("Add", disabled=False, key="dotenv", value=default)

    st.text_input(
        "OpenAI API Key",
        type="password",
        help="Enter your OpenAI API key here.",
        placeholder="sk-...",
    )
    st.text_input(
        "Replicate API Key",
        type="password",
        help="Enter your Replicate API key here.",
        placeholder="r8_...",
    )
    st.text_input(
        "Moondream API Key",
        type="password",
        help="Enter your Moondream API key here.",
        placeholder="...",
    )

    st.button(
        "Install", help="Install all the selected components.", use_container_width=True
    )


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
    st.divider()
    st.subheader("Dependencies")
    check_dependencies_output = st.button(
        "Check dependencies",
        help="Check if all dependencies are installed.",
    )
    if check_dependencies_output:
        container = st.container(border=True)
        with container:
            st.json(st.session_state.dependencies_summary)


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
