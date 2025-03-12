#!/bin/bash

ORIGINAL_PERMISSIONS=$(stat -c "%a" /dev/uinput)
ORIGINAL_OWNER=$(stat -c "%u:%g" /dev/uinput)

reset_permissions() {
    sudo chmod $ORIGINAL_PERMISSIONS /dev/uinput
    sudo chown $ORIGINAL_OWNER /dev/uinput
}
trap reset_permissions EXIT

dir=$(dirname "${BASH_SOURCE[0]}")

sudo chmod 666 /dev/uinput
python3 "$dir/client.py"
python3 /path/to/your/script.py
reset_permissions
