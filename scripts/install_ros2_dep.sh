#!/bin/bash

cd ~/workspace/RUCHE/ros2_pkg
source /opt/ros/jazzy/setup.bash
rosdep update
rosdep install --from-paths src/ -y --ignore-src

# EOF

