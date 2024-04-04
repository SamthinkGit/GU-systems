#!/bin/bash

dir=$(dirname "${BASH_SOURCE[0]}")
SOURCE=$(realpath $dir/..)
export PYTHONPATH=$PYTHONPATH:$SOURCE

# Colcon Build Ignores
PYTHONWARNINGS="ignore:easy_install command is deprecated,ignore:setup.py install is deprecated,ignore:" export PYTHONWARNINGS

# ROS2 sources
workspaces_dir=$SOURCE/gusysros/workspaces

for dir in "$workspaces_dir"/*; do
    if [ -d "$dir" ]; then
        source "$dir/install/setup.sh"
    fi
done
