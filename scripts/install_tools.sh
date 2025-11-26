#!/bin/bash

set -e
echo SEARCH FOR THIS STRING!
# Launch ros2 dependency installation script
/workspaces/RUCHE/scripts/install_ros2_dep.sh

# Install prettier
npm install -g prettier

# Print Operating System name and version
uname -a


# EOF
