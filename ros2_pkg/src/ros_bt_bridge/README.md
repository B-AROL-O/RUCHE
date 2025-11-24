# ros_bt_bridge package

Simple subscriber that publishes a topic over bluetooth.

Prequisites:

Install your dependencies

```sh
rosdep install --from-paths src -r -y
```

Find the servide uuid

```sh
bluetoothctl
```

After you manually connect to the device

```sh
menu gatt
list-attributes
```

Look for the UUID relative to `Nordic UART TX` (or something related to serial data transmission)

Launch with:

```sh
ros2 launch ros_bt_bridge ros_bt_bridge --ros-args -p uuid:=XXX -p topic:=/hardware_interface_command -p device_name:=XXX"
```

replace with the service UUID you want to transmit the messages to, and the name of the device to find.

> Note: The topic is /hardware_interface_command because it is what is generated in the hardware interface.
