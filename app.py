import gradio as gr
import threading
import time

# ===========================
# MOCK / ROS2 SWITCH
# ===========================
MOCK_ROS = True  # set to False if running with real ROS2

if MOCK_ROS:
    # ===========================
    # Mock ROS classes
    # ===========================
    class TwistStamped:
        def __init__(self):
            class Twist:
                def __init__(self):
                    self.linear = type("obj", (), {"x": 0})()
                    self.angular = type("obj", (), {"z": 0})()
            self.twist = Twist()

    class Node:
        def __init__(self, name):
            self.name = name
        def create_publisher(self, msg_type, topic, queue_size):
            return self
        def publish(self, msg):
            print(f"[MOCK ROS] Publishing: linear={msg.twist.linear.x}, angular={msg.twist.angular.z}")
        def create_rate(self, hz):
            class Rate:
                def sleep(self): time.sleep(0.1)
            return Rate()
        def get_clock(self):
            class Clock:
                def now(self):
                    class Time:
                        def seconds_nanoseconds(self):
                            t = time.time()
                            seconds = int(t)
                            nanoseconds = int((t - int(t)) * 1e9)
                            return (seconds, nanoseconds)
                    return Time()
            return Clock()

        def destroy_node(self): pass

    class rclpy:
        @staticmethod
        def init(): pass
        @staticmethod
        def spin(node): pass

else:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import TwistStamped

# ===========================
# ROS2 Node & Publisher
# ===========================
class RobotPublisher(Node):
    def __init__(self):
        super().__init__('gradio_robot_controller')
        self.pub = self.create_publisher(TwistStamped, '/base_controller/cmd_vel', 10)

    def send_twist(self, linear_x=0.0, angular_z=0.0, duration=1.0):
        twist_msg = TwistStamped()
        twist_msg.twist.linear.x = linear_x
        twist_msg.twist.angular.z = angular_z

        sec, nsec = self.get_clock().now().seconds_nanoseconds()
        start_time = sec + nsec / 1e9

        rate = self.create_rate(10)
        while True:
            sec, nsec = self.get_clock().now().seconds_nanoseconds()
            current_time = sec + nsec / 1e9
            if current_time - start_time >= duration:
                break
            self.pub.publish(twist_msg)
            rate.sleep()

        # stop robot
        twist_msg.twist.linear.x = 0.0
        twist_msg.twist.angular.z = 0.0
        self.pub.publish(twist_msg)
        return f"Command sent: linear={linear_x}, angular={angular_z}, duration={duration:.2f}s"

# ===========================
# Initialize ROS / Mock
# ===========================
rclpy.init()
robot_node = RobotPublisher()
ros_thread = threading.Thread(target=rclpy.spin, args=(robot_node,), daemon=True)
ros_thread.start()

# ===========================
# Gradio sync functions
# ===========================
def move_forward_sync(distance_meters: float):
    speed = 0.7  # m/s
    duration = distance_meters / speed
    return robot_node.send_twist(linear_x=speed, angular_z=0.0, duration=duration)

def turn_left_sync():
    angular_speed = 1.4  # rad/s
    duration = 1.0       # adjust for 90° turn
    return robot_node.send_twist(linear_x=0.0, angular_z=angular_speed, duration=duration)

def turn_right_sync():
    angular_speed = -1.4
    duration = 1.0
    return robot_node.send_twist(linear_x=0.0, angular_z=angular_speed, duration=duration)

# ===========================
# Gradio UI
# ===========================
with gr.Blocks(title="Robot Control MCP (ROS2 / Mock)") as demo:
    with gr.Tab("Robot Control MCP"):
        gr.Markdown("## Robot Control")

        robot_output = gr.Textbox(label="Result")

        distance_input = gr.Number(label="Forward Distance (meters)", value=1.0)

        forward_btn = gr.Button("Move Forward", variant="primary")
        forward_btn.click(
            fn=move_forward_sync,
            inputs=[distance_input],
            outputs=robot_output,
            api_name="move_forward",
        )

        left_btn = gr.Button("Turn Left (90°)")
        right_btn = gr.Button("Turn Right (90°)")

        left_btn.click(
            fn=turn_left_sync,
            inputs=[],
            outputs=robot_output,
            api_name="turn_left",
        )

        right_btn.click(
            fn=turn_right_sync,
            inputs=[],
            outputs=robot_output,
            api_name="turn_right",
        )

# ===========================
# Launch
# ===========================
if __name__ == "__main__":
    print("Starting Gradio/MCP server...")
    demo.launch(mcp_server=True)
