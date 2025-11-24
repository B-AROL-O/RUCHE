# ros_bt_bridge package

Simple subscriber that publishes a topic over bluetooth.

Prequisites:

<!-- On linux, pair your bluetooth device and bind it:

```sh
sudo rfcomm bind rfcomm0 <MAC_ADDRESS>
```

Then add your user to the dialout group

```sh
sudo usermod -a -G dialout $USER
``` -->

Install your dependencies

```sh
rosdep install --from-paths src -r -y
```

Launch with:

```sh
ros2 launch ros_bt_bridge ros_bt_bridge.py address:="XX:XX:XX:XX:XX:XX" topic:=/XXX"
```

replace with the device address you want to transmit the messages to.
