# Ros2 control package for RUCHE

This ROS2 package uses a `ros2_control' controller to control a simulated or real robot for the RUCHE project.

> NOTE! The package has been built on ROS2 jazzy and uses Gazebo harmonic for simulation purposes

## How to run

Clone the repository and cd to the `ruche_ros2_control` folder.

Install the dependencies:

### Dependencies

Make sure to install the following packages before you go ahead:

- ROS2 - follow the [official guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- Gazebo - follow the [official guide](https://gazebosim.org/docs/harmonic/ros_installation/#summary-of-compatible-ros-and-gazebo-combinations)
- Rviz2 - `ros-${ROS_DISTRO}-rviz2`
- Rosdep - `apt-get install python3-rosdep`

Run rosdep to check the other dependencies:

```sh
sudo rosdep init
rosdep update
rosdep install --from-paths src/ -y --ignore-src
```

Once you are done, you can move on to launch the package.

### Launching

You can launch the package with the following commands:

Source your ros installation

```sh
source /opt/ros2/jazzy/setup.bash
```

build the package

```sh
colcon build --symlink-install --cmake-clean-cache --packages-select ruche_ros2_control
source install/setup.bash
```

Launch!

```sh
ros2 launch ruche_ros2_control diffelegoo.launch.py robot_name:=diffelegoo simulate:=true
```

> Note: If you want to run the package for the real hardware, change the argument to `simulate:=false`.
> in this case, the robot `diffelegoo` is simulated. If multiple robots are added, use the proper one changing the name argument
