#!/bin/bash

dir=$(dirname "${BASH_SOURCE[0]}")
SOURCE=$(realpath $dir/..)
export PYTHONPATH=$PYTHONPATH:$SOURCE

# Colcon Build Ignores
PYTHONWARNINGS="ignore:easy_install command is deprecated,ignore:setup.py install is deprecated,ignore:"
export PYTHONWARNINGS

# ROS2 sources
workspaces_dir=$SOURCE/execution_layer/rosa/ros2/workspaces

for dir in "$workspaces_dir"/*; do
    if [ -d "$dir" ]; then
        source "$dir/install/setup.sh"
    fi
done

# -------------------------- Some dev-aliases ---------------------------------

# [gutree]: Show the tree of the proyect without redundant files
alias gutree='tree -I interfaces -I install -I log -I __pycache__ -I docs -I build -I __init__.py'

# [jsed]: Prints json files formatted (used in combination with ros2 topic echo)
# Example: ros2 topic echo /tast_registry | jsed
alias jsed="sed -E -e 's/\\\n/\n/g' -e 's/\\\//g' -e 's/\\\"//g'"

# [gureg]: Shows the status of the TaskRegistry in live
alias gureg="watch $SOURCE/scripts/show_registry.sh"

# -----------------------------------------------------------------------------
