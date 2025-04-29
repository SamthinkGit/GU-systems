# flake8: noqa: F841
import streamlit as st
from installers.description import detect_os
from installers.description import InstallerDescription
from src.installation_options import get_cognition_layer_options
from src.installation_options import get_execution_layer_options


def load_selection_tab():
    with st.form(key="easy_install_form", border=False):
        st.write("The following components will be installed for using the ECM.")
        st.write("Usually, you want to select only the latest version.")
        st.divider()

        st.subheader("Components")
        c1, c2 = st.columns(2)
        with c1:
            st.write("**Cognition Layer:**")
        with c2:
            cognition_layer = st.selectbox(
                "",
                get_cognition_layer_options(),
                label_visibility="collapsed",
                help="Select the execution layer you want to use. Usually, you want to use only the latest version.",
            )
        c1, c2 = st.columns(2)
        with c1:
            st.write("**Execution Layer:**")
        with c2:
            execution_layer = st.selectbox(
                "",
                get_execution_layer_options(),
                help="Select the execution layer you want to use. Usually, you want to use only the latest version.",
                label_visibility="collapsed",
            )
        install_with_conda = st.checkbox(
            "Install in a conda environment (conda must be installed)",
        )

        st.divider()
        st.subheader("API keys")
        st.markdown(
            """
            The following API keys will be used for using AI services.\n
            > For information on how to get the API keys, please refer to the
            > `Help` tab.
            """
        )

        openai_api_key = st.text_input(
            "OpenAI API Key",
            type="password",
            help="Enter your OpenAI API key here.",
            placeholder="sk-...",
        )
        replicate_api_key = st.text_input(
            "Replicate API Key",
            type="password",
            help="Enter your Replicate API key here.",
            placeholder="r8_...",
        )
        moondream_api_key = st.text_input(
            "Moondream API Key",
            type="password",
            help="Enter your Moondream API key here.",
            placeholder="...",
        )

        already_have_api_keys = st.checkbox(
            "I already have the API keys in my environment variables.",
            help="If you check this box, the API keys will be read from your environment variables. Otherwise, the api fields will be used.",
        )

        submit_button = st.form_submit_button(label="Install")

    if submit_button:
        st.session_state.page = "_confirm_install"
        description = InstallerDescription(
            cognition_layers=[cognition_layer, "base"],
            execution_layers=[execution_layer],
            ecm_dependencies=["base", "devel"],
            precommit=False,
            api_keys=(
                {
                    "OPENAI_API_KEY": openai_api_key,
                    "REPLICATE_API_KEY": replicate_api_key,
                    "MOONDREAM_API_KEY": moondream_api_key,
                }
                if not already_have_api_keys
                else {}
            ),
            os=detect_os(),
            install_with_conda=install_with_conda,
            conda_path="ecm",
            git_pull=False,
            setup_python_path=True,
        )
        st.session_state.installation_description = description
        st.rerun()


def load_help_tab():
    st.title("Help - Frequently Asked Questions")

    with st.expander("🔹 Why do I need 3 API keys?"):
        st.markdown(
            """
        You need three API keys to fully use ECM functionalities:

        - **OpenAI API Key:**
          Required to make requests to GPT models (typically `gpt-4o-mini`) for general cognition tasks.

        - **Replicate API Key:**
          Required to access **MolMo**, an open-source vision model.
          (Newer ECM agents need MolMo; older versions may work without this key.)

        - **Moondream API Key:**
          Used to interact with **Moondream 0.7B**, a faster vision model for quick visual analysis inside ECM.
        """
        )

    with st.expander("🔹 Where can I get each API key?"):
        st.markdown(
            """
        You can obtain the required API keys from the following links:

        - [OpenAI API Key](https://platform.openai.com/api-keys)
        - [Replicate API Key](https://replicate.com/account/api-tokens)
        - [Moondream API Key](https://moondream.ai/c/cloud/api-keys)

        When accessing these pages, you will be prompted to **create an account** if you don't already have one.
        Once logged in, you will be able to generate and copy the necessary token.

        ---
        > **Note:**
        > If you do not have an API key and you are a user with sufficient access rights,
        > you can request **temporary API keys** to test the system by contacting the host at:
        > **sebastianmayorquin@gmail.com**
        """
        )

    with st.expander("🔹 Does ECM support tracing with LangSmith?"):
        st.markdown(
            """
        ECM also supports **LangSmith tracing** if you set the corresponding API key in your environment variables.

        ⚡ However, **automatic installation** for LangSmith tracing is **not yet available** through this installer.
        """
        )

    with st.expander("🔹 What happens if I choose to install in a Conda environment?"):
        st.markdown(
            """
        If you choose the option to install in a **Conda environment**:

        - A new environment named **`ecm`** will be created automatically.
        - The environment will use **Python 3.10**.
        - All required dependencies for ECM will be installed inside this environment.

        After installation, you will be able to activate it manually by running:

        ```bash
        conda activate ecm
        ```

        **Note:** Conda must be installed on your system before using this option.
        """
        )

    st.info(
        "If you have further questions, please check the official documentation or contact support."
    )


def load_standard_tab():
    st.header("Default Installation")
    config, help = st.tabs(["Selection", "Help"])
    with config:
        load_selection_tab()
    with help:
        load_help_tab()
