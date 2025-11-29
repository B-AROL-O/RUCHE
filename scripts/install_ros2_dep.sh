#!/bin/bash

cd $(dirname $0)/../ros2_pkg
apt-get update
source /opt/ros/jazzy/setup.bash
rosdep update
rosdep install --from-paths src/ -y --ignore-src

# EOF

