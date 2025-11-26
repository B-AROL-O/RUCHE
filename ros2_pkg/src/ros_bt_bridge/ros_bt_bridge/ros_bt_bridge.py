
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import asyncio
from bleak import BleakClient, BleakScanner
import threading
import json


def format_string(msg: str) -> str:
    data = json.loads(msg)
    return f"vl:{data['v_l']:05.2f};vr:{data['v_r']:05.2f}"


class RosBluetoothBridge(Node):
    def __init__(self):
        super().__init__('ros_bluetooth_bridge')

        self.dev_ = None
        self.running_ = False
        self.loop_ = asyncio.new_event_loop()
        # self.char_uuid = "0000ffe1-0000-1000-8000-00805f9b34fb"
        self.client_ = None

        # Set thread where to execute asynk tasks
        threading.Thread(target=self.loop_.run_forever, daemon=True).start()

        # Declare parameters
        self.declare_parameter('topic', '/robot_description')
        self.declare_parameter('device_name', 'ESP32-BLE')
        self.declare_parameter('uuid', '6e400002-b5a3-f393-e0a9-e50e24dcca9e')

        # Get arguments
        topic = self.get_parameter('topic')\
            .get_parameter_value().string_value
        device_name = self.get_parameter('device_name')\
            .get_parameter_value().string_value
        self.char_uuid_ = self.get_parameter('uuid')\
            .get_parameter_value().string_value

        # Search and connect to the device. Wait until the config is finished
        config = asyncio.run_coroutine_threadsafe(
            self.configure(device_name),
            self.loop_
        )
        config.result()

        self.subscription = self.create_subscription(
            String,
            topic,
            self.listener_callback,
            10)

        self.get_logger().info(f"Subscribed to: {topic}")

    async def configure(self, name: str):
        self.get_logger().info(f"Looking for the device: {name}")
        self.dev_ = await BleakScanner.find_device_by_name(name, 10)
        if self.dev_ is None:
            self.get_logger().fatal("ERROR: Cannot find the device!")
        else:
            self.get_logger().info("Device found!")
            self.client_ = BleakClient(self.dev_)
            await self.client_.connect()
            if self.client_.is_connected:
                self.get_logger().info("Device connected!")
            else:
                self.get_logger().fatal("Device NOT connected!")

    async def send_string(self, string):
        if self.client_.is_connected:
            await self.client_.write_gatt_char(
                self.char_uuid_,
                string.encode()
            )
        else:
            self.get_logger().fatal("Device NOT connected!")
        self.running_ = False

    def listener_callback(self, msg):
        formatted_msg = format_string(msg.data)
        if (not self.running_):
            self.get_logger().info(
                f"Sending: {formatted_msg}"
            )
            self.running_ = True
            asyncio.run_coroutine_threadsafe(
                self.send_string(formatted_msg),
                self.loop_
            )
        else:
            # self.get_logger().info("Message lost")
            pass


def main(args=None):
    rclpy.init(args=args)
    node = RosBluetoothBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
