#!/bin/bash

set -e

sudo apt-get update

cd "$(dirname \"$0\")/../ros2_pkg" || exit

# shellcheck source=/dev/null
source /opt/ros/jazzy/setup.bash

rosdep update
rosdep install --from-paths src/ -y --ignore-src

# EOF
