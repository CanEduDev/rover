#!/bin/bash
set -eo pipefail

# setup gateway env
# shellcheck disable=SC1091
source "./install/setup.bash"

ros2 launch gateway gateway.launch.py "$@"
