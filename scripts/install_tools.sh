#!/bin/bash

set -e

# Install prettier
npm install -g prettier

# Print Operating System name and version
# Launch ros2 dependency installation script
/workspaces/RUCHE/scripts/install_ros2_dep.sh

uname -a

# EOF
