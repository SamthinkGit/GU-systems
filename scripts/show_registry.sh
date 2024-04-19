#!/bin/sh
alias jsed="sed -E -e 's/\\\n/\n/g' -e 's/\\\//g' -e 's/\\\"//g'"
ros2 topic echo /task_registry -f --once | jsed
