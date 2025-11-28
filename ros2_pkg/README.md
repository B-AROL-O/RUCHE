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

build the packages

```sh
colcon build --symlink-install --cmake-clean-cache
source install/setup.bash
```

Launch!

```sh
ros2 launch ruche_ros2_control robot.launch.py robot_name:=diffelegoo simulate:=true
```

> Note: If you want to run the package for the real hardware, change the argument to `simulate:=false`.
> in this case, the robot `diffelegoo` is simulated. If multiple robots are added, use the proper one changing the name argument
>
> It might be necessary to launch some additional nodes depending on the robot you are using:
>
> - [ELEGOO robot](#elegoo-robot)

## Testing

After launching the robot of interest, you can test the controller and communication with the following twist command:

```sh
ros2 topic pub /base_controller/cmd_vel geometry_msgs/msg/TwistStamped "{header: auto, twist: {linear: {x: 0.70}, angular: {z: 1.4}}}"
```

You should see the robot (real or simulated) moving in a circular pattern

## Robots

The following section assumes you have already built the packages without errors following the previous instructions.

It is assumed that to launch the next commands you are in the path `RUCHE/ros2_pkg`

### ELEGOO robot

The robot is a 4-wheel robot kit controlled via bluetooth.

Since it is possible to directly control only two of the wheels, it has been modeled as a differental drive robot.

#### ELEGOO launch commands

Open two terminals and run the following commands to launch the controller and brigde

In terminal 1:

```sh
ros2 launch ruche_ros2_control robot.launch.py robot_name:=diffelegoo simulate:=false
```

In terminal 2:

```sh
ros2 run ros_bt_bridge ros_bt_bridge --ros-args -p topic:=/hardware_interface_command -p addevice_name:=[DEVICE_NAME] -p uuid:=[UUID]
```

For more details on the bridge, consult the dedicated [Readme](src/ros_bt_bridge/README.md)


