#!/bin/bash

set -e

# Launch ros2 dependency installation script
"$(dirname $0)/install_ros2_dep.sh"

# Install prettier
npm install -g prettier

# Print Operating System name and version
uname -a

# EOF
