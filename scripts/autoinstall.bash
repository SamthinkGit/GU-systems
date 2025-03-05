#!/bin/bash

GREEN="\033[32m"
BOLD="\033[1m"
RESET="\033[0m"
YELLOW="\033[33m"
RED="\033[1;31m"

PYTHON_VERSION="3.11"
CONDA_ENV_NAME="ecm"
SOURCE_CONFIG_PATH="/scripts/dev_setup.sh"

title() {
  echo -e "\033[1;34m==================== $1 ====================\033[0m"
}

log_step() {
    echo -e "[${YELLOW}ECM${RESET}] $1"
}

ask_question() {
    echo -ne "•${BOLD} ${GREEN}$1: ${RESET}"
    read -r response
}

is_yes() {
    case "$1" in
        y|Y|yes|YES|Yes) return 0 ;;
        *) return 1 ;;
    esac
}


set -e
handle_error() {
    # Since this file is sourced, it cannot be exited without closing the console
    # Then, ask the user to stop the source

    echo -e "${RED}ERROR: Script failed on line $1. Stop${RESET}"
    echo -e "${RED}Use Ctrl+C to exit...${RESET}"
    while true; do sleep 1; done
}

trap 'handle_error $LINENO' ERR


title "GU-Systems Autoinstaller"
ask_question "Use a Conda environment? (y/n)"

if [ "$response" = "y" ] || [ "$response" = "Y" ] || [ "$response" = "yes" ]; then
    log_step "Creating Conda environment..."
    for i in $(seq ${CONDA_SHLVL}); do
        conda deactivate
    done

    env_created="false"

    if conda info --envs | grep -q $CONDA_ENV_NAME; then
        ask_question "The ${CONDA_ENV_NAME} environment already exists. Do you want to replace it? (y/n)"
        if is_yes $response; then
            log_step "Removing existing Conda environment..."
            conda env remove -y --name $CONDA_ENV_NAME
        else
            env_created="true"
        fi
    fi

    if [ "$env_created" = "false" ]; then
        conda create -y --name $CONDA_ENV_NAME python=$PYTHON_VERSION
        log_step "Environment $CONDA_ENV_NAME created"
    fi

    conda activate $CONDA_ENV_NAME
fi


log_step "Installing Dependencies..."

dir=$(dirname "${BASH_SOURCE[0]}")
repository_dir=$(realpath $dir/..)

pip install -r "$repository_dir/requirements.txt" --disable-pip-version-check
pip install python-uinput
pip install pre-commit

log_step "Installing Pre-Commit checkers..."
pre-commit install

log_step "Checking API KEYS..."
if [ "$OPENAI_API_KEY" =  "" ]; then
    ask_question "OPENAI_API_KEY variable not settled. Do you want to add it? (y/n)"
    if is_yes $response; then
        ask_question "Please provide an API KEY. It can be obtained from https://openai.com/index/openai-api/ and look like: sk-...
OPENAI_API_KEY"

        touch "$repository_dir/.env"
        echo "OPENAI_API_KEY=$response" > "'$repository_dir/.env'"
    fi
fi

log_step "Sourcing configs..."

sourcer_path=""

if [ -e "$HOME/.zhr" ]; then
    sourcer_path="$HOME/.zshrc"
fi

if [ -e "$HOME/.bashrc" ]; then
    sourcer_path="$HOME/.bashrc"
fi

if ! [ "$sourcer_path" = "" ]; then
    if ! grep -q -E "source ${repository_dir}${SOURCE_CONFIG_PATH}" "$sourcer_path"; then
        echo "source ${repository_dir}${SOURCE_CONFIG_PATH}" >> $sourcer_path
        echo "Configs saved, restart the console for applying changes."
    fi
else
    log_step "[WARNING] $HOME/.bashrc or $HOME/.zshrc are not valid paths for auto-sourcing config. Skip"
fi

title "Installation Completed"
set +x
trap - ERR
