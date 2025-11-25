import gradio as gr
import asyncio
import threading
from huggingface_hub import InferenceClient

from robot import robot_instance


# ============================================================
# Dedicated async event loop for robot operations
# ============================================================

robot_loop = asyncio.new_event_loop()

def start_robot_loop(loop):
    asyncio.set_event_loop(loop)
    loop.run_forever()

threading.Thread(target=start_robot_loop, args=(robot_loop,), daemon=True).start()

def run_async_task(coro):
    """Runs an async coroutine on the robot loop and returns the result."""
    future = asyncio.run_coroutine_threadsafe(coro, robot_loop)
    return future.result()


# ============================================================
# Sync wrappers for Gradio + MCP exposure
# ============================================================

def get_telemetry_sync():
    state = robot_instance.get_state()
    return (
        f"Telemetry: X={state['x']}, Y={state['y']}, "
        f"Heading={state['heading']}°, Distance Traveled={state['distance_traveled']:.2f}"
    )

def move_forward_sync(distance: float):
    if distance <= 0:
        return "Error: Distance must be positive."
    
    try:
        result = run_async_task(robot_instance.move_forward(distance))
        state = result["current_state"]
        return (
            f"Moved forward {distance} meters.\n"
            f"State: X={state['x']}, Y={state['y']}, Heading={state['heading']}°"
        )
    except Exception as e:
        return f"Error: {e}"

def turn_left_sync():
    result = run_async_task(robot_instance.turn_left())
    state = result["current_state"]
    return (
        f"Left turn completed.\n"
        f"State: X={state['x']}, Y={state['y']}, Heading={state['heading']}°"
    )

def turn_right_sync():
    result = run_async_task(robot_instance.turn_right())
    state = result["current_state"]
    return (
        f"Right turn completed.\n"
        f"State: X={state['x']}, Y={state['y']}, Heading={state['heading']}°"
    )


# ============================================================
# Chat function
# ============================================================

def respond(
    message,
    history: list[dict[str, str]],
    system_message,
    max_tokens,
    temperature,
    top_p,
    hf_token: gr.OAuthToken,
):
    """
    For more information on `huggingface_hub` Inference API support, please check the docs: https://huggingface.co/docs/huggingface_hub/v0.22.2/en/guides/inference
    """
    
    client = InferenceClient(token=hf_token.token, model="openai/gpt-oss-20b")

    messages = [{"role": "system", "content": system_message}]
    messages.extend(history)
    messages.append({"role": "user", "content": message})

    buffer = ""

    for msg in client.chat_completion(
        messages,
        max_tokens=max_tokens,
        temperature=temperature,
        top_p=top_p,
        stream=True,
    ):
        token = msg.choices[0].delta.content or ""
        buffer += token
        yield buffer


# ============================================================
# Gradio UI + MCP tools
# ============================================================

"""
For information on how to customize the ChatInterface, peruse the gradio docs: https://www.gradio.app/docs/gradio/chatinterface
"""

chatbot = gr.ChatInterface(
    respond,
    type="messages",
    additional_inputs=[
        gr.Textbox(value="You are a friendly Chatbot.", label="System message"),
        gr.Slider(1, 2048, value=512, step=1, label="Max new tokens"),
        gr.Slider(0.1, 4.0, value=0.7, step=0.1, label="Temperature"),
        gr.Slider(0.1, 1.0, value=0.95, step=0.05, label="Top-p"),
    ],
)

with gr.Blocks(title="Robot Control and Chatbot MCP") as demo:
    with gr.Sidebar():
        gr.LoginButton()

    with gr.Tab("Chatbot"):
        chatbot.render()

    with gr.Tab("Robot Control MCP"):
        gr.Markdown("## Robot Control and Telemetry")

        robot_output = gr.Textbox(label="Result / Telemetry")

        telemetry_btn = gr.Button("Update Telemetry", variant="secondary")
        telemetry_btn.click(
            fn=get_telemetry_sync,
            inputs=[],
            outputs=robot_output,
            api_name="get_telemetry",
        )

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


# ============================================================
# Launch
# ============================================================

if __name__ == "__main__":
    print("Starting Gradio/MCP server...")
    print(f"DEBUG: Gradio version: {gr.__version__}")
    demo.launch(
        mcp_server=True,
        # share=False,
        # server_name="localhost",
        # server_port=7860,
    )
