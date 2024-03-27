#!/bin/bash

dir=$(dirname "${BASH_SOURCE[0]}")
SOURCE=$(realpath $dir/..)
export PYTHONPATH=$PYTHONPATH:$SOURCE