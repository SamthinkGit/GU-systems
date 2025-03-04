#!/bin/bash

# ----- Some fancy functions ----
function title {
  echo -e "\033[1;34m==================== $1 ====================\033[0m"
}

function step {
  echo -e "\033[1;33m- $1...\033[0m"
}


# ----- MAIN -----

workspaces_dir=$SOURCE/execution_layer/rosa/ros2/workspaces

for dir in "$workspaces_dir"/*; do
    if [ -d "$dir" ]; then
        step "Building $dir"
        cd $dir
        colcon build --symlink-install
        source './install/setup.sh'
    fi
done
