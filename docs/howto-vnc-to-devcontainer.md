# HOWTO Use VNC to interact with RUCHE Dev Container

## Introduction

This document explains how to use VNC to interact with the Graphical Applications of the RUCHE Development Environmnent deployed inside a Dev Container.

In the following example:

- The VNC client runs on `delta` (Ubuntu Desktop 24.04.x)
- The RUCHE Development Environments runs in a Dev Container in Visual Studio Code on `labai-ubnt02` (Ubuntu Server 24.04.x)

## Step-by-step instructions

<!-- (2025-12-01 08:39 CET) -->

Logged in as `gmacario@delta`:

- Type the following command to enable remote X connections (TODO: is this required?):

  ```bash
  xhost +
  ```

- Launch Visual Studio Code
- Inside Visual Studio Code, type **Ctrl-Shift-P** to open the Command Palette and select
  > Remote-SSH: Connect to Host...
  > to connect to `gmacario@labai-ubnt02` via SSH

Once Visual Studio Code is running as `gmacario@labai-ubnt02`:

- Open the folder where you cloned the RUCHE repository
- Repen the project in a Dev Container
- Open a bash Terminal

Logged in as `vscode@labai-ubnt02` (Dev Container: RUCHE @ labai-ubnt02), type the following commands:

```bash
git checkout feat/gmacario-devcontainer-ros
git pull --all --prune
colcon build
```

Logged in as `gmacario@delta`, type the following command:

```bash
remmina -c vnc://vscode@localhost:5901
```

A window will open with the request for the VNC password:

![2025-11-30-vnc-devcontainer01.png](images/2025-11-30-vnc-devcontainer01.png)

Enter VNC password: `vscode`

As a result, a Linux desktop will be displayed:

![2025-11-30-vnc-devcontainer01.png](images/2025-11-30-vnc-devcontainer01.png)

Right click the **Terminator** icon > Open

Type the following commands inside the terminal:

```bash
ros2 topic list
source install/setup.bash

# Launch the RUCHE nodes and the Gazebo Sim
ros2 launch ruche_ros2_control robot.launch.py \
    robot_name:=diffelegoo simulate:=true
```

To make the robot move, open a second terminal and type the following commands:

```bash
source install/setup.bash

# Send a Twist command to test the controller and communication
ros2 topic pub /base_controller/cmd_vel geometry_msgs/msg/TwistStamped \
    "{header: auto, twist: {linear: {x: 0.70}, angular: {z: 1.4}}}"
```

Screenshot:

![2025-11-30-vnc-devcontainer03.png](images/2025-11-30-vnc-devcontainer03.png)

<!-- EOF -->
