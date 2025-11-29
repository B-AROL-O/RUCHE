#!/usr/bin/env python3
import asyncio
import threading
import math
import time

import gradio as gr
from huggingface_hub import InferenceClient

# ===============================
# ROS2 / Simulation Controller
# ===============================
try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import TwistStamped

    class RobotController(Node):
        def __init__(self):
            super().__init__("chatbot_robot_controller")
            self.publisher = self.create_publisher(TwistStamped, "/base_controller/cmd_vel", 10)

        def publish_once(self, linear_x=0.0, angular_z=0.0):
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.twist.linear.x = float(linear_x)
            msg.twist.angular.z = float(angular_z)
            self.publisher.publish(msg)
            print(f"[ROS2] Published linear={linear_x}, angular={angular_z}")
            return {"linear_x": linear_x, "angular_z": angular_z}

    ROS_ENABLED = True
except ImportError:
    class RobotController:
        def publish_once(self, linear_x=0.0, angular_z=0.0):
            print(f"[SIM] Publishing linear={linear_x}, angular={angular_z}")
            return {"linear_x": linear_x, "angular_z": angular_z}

    ROS_ENABLED = False

# ===============================
# Command Manager
# ===============================
class CommandManager:
    MAX_LINEAR_SPEED = 0.5
    MAX_ANGULAR_SPEED = 1.0

    def __init__(self, controller):
        self.controller = controller
        self._lock = asyncio.Lock()
        self._active_task = None
        self._cancel_event = None
        self.state = {"x":0.0, "y":0.0, "heading":0.0}

    async def move(self, distance: float):
        async with self._lock:
            speed = min(0.3, self.MAX_LINEAR_SPEED)
            direction = 1.0 if distance >= 0 else -1.0
            duration = abs(distance)/speed
            cancel_event = asyncio.Event()
            self._cancel_event = cancel_event
            self._active_task = asyncio.create_task(
                self._publish_continuous(speed*direction, 0.0, duration, cancel_event)
            )
            await self._active_task
            self._active_task = None
            self._cancel_event = None
            rad = math.radians(self.state["heading"])
            self.state["x"] += distance * math.cos(rad)
            self.state["y"] += distance * math.sin(rad)
            return f"Moved {distance} meters."

    async def rotate(self, angle: float):
        async with self._lock:
            rot_speed = min(0.785, self.MAX_ANGULAR_SPEED)  # ~45°/s
            direction = 1.0 if angle >= 0 else -1.0
            duration = abs(angle)/45.0
            cancel_event = asyncio.Event()
            self._cancel_event = cancel_event
            self._active_task = asyncio.create_task(
                self._publish_continuous(0.0, rot_speed*direction, duration, cancel_event)
            )
            await self._active_task
            self._active_task = None
            self._cancel_event = None
            self.state["heading"] = (self.state["heading"] + angle) % 360
            return f"Rotated {angle} degrees."

    async def _publish_continuous(self, linear, angular, duration, cancel_event):
        start = time.time()
        rate = 10.0
        period = 1.0/rate
        while True:
            if cancel_event.is_set():
                self.controller.publish_once(0.0, 0.0)
                return
            elapsed = time.time() - start
            if elapsed >= duration:
                self.controller.publish_once(0.0, 0.0)
                return
            self.controller.publish_once(linear, angular)
            await asyncio.sleep(period)

    async def stop(self):
        async with self._lock:
            if self._active_task and not self._active_task.done():
                self._cancel_event.set()
                await asyncio.sleep(0.1)
            self.controller.publish_once(0.0, 0.0)
            return "Robot stopped."

    def get_state(self):
        return self.state.copy()

# ===============================
# Setup Controller & Loop
# ===============================
ros_loop = asyncio.new_event_loop()
controller = RobotController()
command_manager = CommandManager(controller)

# Start ROS thread only if ROS2 is available
if ROS_ENABLED:
    import rclpy
    def ros_spin():
        rclpy.spin(controller)
    threading.Thread(target=ros_spin, daemon=True).start()

def run_async(coro):
    future = asyncio.run_coroutine_threadsafe(coro, ros_loop)
    return future.result()

# ===============================
# Chatbot handler
# ===============================
def respond(message, history, system_message, max_tokens, temperature, top_p, hf_token: gr.OAuthToken):
    """
    Interpret user text as robot commands.
    Supported: move X meters, rotate X degrees, stop, status
    """
    text = message.lower()
    result = ""
    try:
        import re
        if "move" in text:
            m = re.search(r"(-?\d+\.?\d*)", text)
            if m:
                distance = float(m.group(1))
                result = run_async(command_manager.move(distance))
            else:
                result = "Specify distance in meters. E.g., 'move 2'"
        elif "rotate" in text or "gira" in text:
            m = re.search(r"(-?\d+\.?\d*)", text)
            if m:
                angle = float(m.group(1))
                result = run_async(command_manager.rotate(angle))
            else:
                result = "Specify angle in degrees. E.g., 'rotate 90'"
        elif "stop" in text:
            result = run_async(command_manager.stop())
        elif "status" in text:
            state = command_manager.get_state()
            result = f"State: X={state['x']:.2f}, Y={state['y']:.2f}, Heading={state['heading']:.0f}°"
        else:
            # fallback to LLM
            client = InferenceClient(token=hf_token.token, model="openai/gpt-oss-20b")
            messages = [{"role": "system", "content": system_message}]
            messages.extend(history)
            messages.append({"role": "user", "content": message})
            buffer = ""
            for msg in client.chat_completion(messages, max_tokens=max_tokens,
                                              temperature=temperature, top_p=top_p, stream=True):
                token = msg.choices[0].delta.content or ""
                buffer += token
                yield buffer
            return
    except Exception as e:
        result = f"Error: {e}"
    yield result

# ===============================
# Launch Gradio
# ===============================
chatbot = gr.ChatInterface(
    fn=respond,
    additional_inputs=[
        gr.Textbox(value="You are a helpful robot assistant.", label="System message"),
        gr.Slider(1, 2048, value=512, step=1, label="Max new tokens"),
        gr.Slider(0.1, 4.0, value=0.7, step=0.1, label="Temperature"),
        gr.Slider(0.1, 1.0, value=0.95, step=0.05, label="Top-p"),
    ]
)

if __name__ == "__main__":
    chatbot.launch(mcp_server=True)
