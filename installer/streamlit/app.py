import streamlit as st
from src.developer_tab import load_developer_tab
from src.install_tab import load_confirm_install_tab
from src.install_tab import load_install_tab
from src.install_tab import load_installation_success_tab
from src.standard_tab import load_standard_tab


def print_welcome_message_to_terminal():
    box = [
        "╔══════════════════════════════════════════════════════════════════════════╗",
        "║                            DO NOT KILL ME!                               ║",
        "║                                                                          ║",
        "║ 🔥  Installer has been deployed!  🔥                                     ║",
        "║                                                                          ║",
        "║ If the browser does not open automatically, please enter the             ║",
        '║ "Network URL" shown upper ↑ with your preferred browser :D               ║',
        "╚══════════════════════════════════════════════════════════════════════════╝",
    ]

    for line in box:
        print(line)


# flake8: noqa: F841
import streamlit as st


def load_welcome_page():
    st.title("Welcome to the ECM Installer 🚀")

    st.write(
        """This installer will guide you through setting up the Execution Cognition Machine (ECM).
    Please choose the section that best fits your role:"""
    )

    st.divider()

    st.subheader("🔹 Standard User")
    st.write(
        """
    - Install the default components required to use the ECM.
    - Select between different versions of the Cognition and Execution layers.
    """
    )

    st.subheader("🔹 Developer")
    st.write(
        """
    - Install only the client, only the host, or both, depending on your development needs.
    - Advanced configuration options available.
    - Update your existing ECM installation to the latest version.
    """
    )

    st.divider()

    st.info(
        "👈 Please select a section from the sidebar to start your installation or update process."
    )

if __name__ == "__main__":

    pages = ["Welcome", "Standard User", "Developer"]
    if "page" not in st.session_state:
        print_welcome_message_to_terminal()
        st.session_state.page = "Welcome"

    st.set_page_config(
        page_title="ECM Installer", initial_sidebar_state="auto", layout="wide"
    )

    st.sidebar.markdown(
        """
        <h1 style='text-align: center; font-size: 25px;'>
            Installation
        </h1>""",
        unsafe_allow_html=True,
    )

    for page in pages:
        if st.sidebar.button(page, use_container_width=True):
            st.session_state.page = page

    if st.session_state.page == "Welcome":
        load_welcome_page()
    if st.session_state.page == "Standard User":
        load_standard_tab()
    if st.session_state.page == "Developer":
        load_developer_tab()
    if st.session_state.page == "_confirm_install":
        load_confirm_install_tab()
    if st.session_state.page == "_install":
        load_install_tab()
    if st.session_state.page == "_installation_success":
        load_installation_success_tab()
