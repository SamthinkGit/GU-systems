#!/bin/bash

# ----- Some fancy functions ----
function title {
  echo -e "\033[1;34m==================== $1 ====================\033[0m"
}

function step {
  echo -e "\033[1;33m- $1...\033[0m"
}

# ----- MAIN -----
dir=$(dirname "${BASH_SOURCE[0]}")
SOURCE=$(realpath $dir/..)
title "GU-Systems"

workspaces_dir=$SOURCE/gusysros/workspaces

for dir in "$workspaces_dir"/*; do
    if [ -d "$dir" ]; then
        step "Building $dir"
        colcon build --symlink-install
        source './install/setup.sh'
    fi
done
