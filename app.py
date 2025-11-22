import gradio as gr
import asyncio
from huggingface_hub import InferenceClient

# Import the robot instance and its logic from mcp_server.py
from mcp_server import robot_instance 

# --- 1. SYNCHRONOUS ROBOT WRAPPER FUNCTIONS (Required for Gradio) ---

def get_telemetry_sync() -> str:
    """Returns the robot state formatted for the interface."""
    state = robot_instance.get_state()
    return f"Telemetry: X={state['x']}, Y={state['y']}, Heading={state['heading']}°, Distance Traveled={state['distance_traveled']:.2f}"

def move_forward_sync(distance: float) -> str:
    """Executes asynchronous move_forward and returns the updated state."""
    if distance <= 0:
        return "Error: Distance must be a positive number."
    
    try:
        # Use asyncio.run to call the asynchronous function
        result = asyncio.run(robot_instance.move_forward(distance))
        state = result['current_state']
        return f"Forward movement of {distance} meters completed.\nState: X={state['x']}, Y={state['y']}, Heading={state['heading']}°"
    except ValueError as e:
        return f"Error: {e}"

def turn_left_sync() -> str:
    """Executes asynchronous turn_left and returns the updated state."""
    result = asyncio.run(robot_instance.turn_left())
    state = result['current_state']
    return f"LEFT turn completed.\nState: X={state['x']}, Y={state['y']}, Heading={state['heading']}°"

def turn_right_sync() -> str:
    """Executes asynchronous turn_right and returns the updated state."""
    result = asyncio.run(robot_instance.turn_right())
    state = result['current_state']
    return f"RIGHT turn completed.\nState: X={state['x']}, Y={state['y']}, Heading={state['heading']}°"


# --- 2. ORIGINAL CHATBOT FUNCTION AND INTERFACE ---

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
    Original function for LLM interaction via the Hugging Face Inference API.
    """
    client = InferenceClient(token=hf_token.token, model="openai/gpt-oss-20b")
    messages = [{"role": "system", "content": system_message}]
    messages.extend(history)
    messages.append({"role": "user", "content": message})
    response = ""

    for message in client.chat_completion(
        messages,
        max_tokens=max_tokens,
        stream=True,
        temperature=temperature,
        top_p=top_p,
    ):
        choices = message.choices
        token = ""
        if len(choices) and choices[0].delta.content:
            token = choices[0].delta.content

        response += token
        yield response


chatbot = gr.ChatInterface(
    respond,
    type="messages",
    additional_inputs=[
        gr.Textbox(value="You are a friendly Chatbot.", label="System message"),
        gr.Slider(minimum=1, maximum=2048, value=512, step=1, label="Max new tokens"),
        gr.Slider(minimum=0.1, maximum=4.0, value=0.7, step=0.1, label="Temperature"),
        gr.Slider(
            minimum=0.1,
            maximum=1.0,
            value=0.95,
            step=0.05,
            label="Top-p (nucleus sampling)",
        ),
    ],
)

# --- 3. BLOCKS STRUCTURE WITH TWO TABS (Chatbot and Robot) ---

with gr.Blocks(title="Robot Control and Chatbot MCP") as demo:
    with gr.Sidebar():
        gr.LoginButton()
    
    # FIRST TAB: The original Chatbot
    with gr.Tab("Chatbot"): 
        chatbot.render()
        
    # SECOND TAB: The Robot Control / MCP Tools Interface
    with gr.Tab("Robot Control MCP"): 
        gr.Markdown("## 🤖 Robot Control and Telemetry Interface")
        robot_output = gr.Textbox(label="Last Action Result / Telemetry")

        # Telemetry
        with gr.Row():
            telemetry_btn = gr.Button("Update Telemetry", variant="secondary")
            # MCP EXPOSURE: api_name="get_telemetry"
            telemetry_btn.click(
                fn=get_telemetry_sync, 
                inputs=[], 
                outputs=robot_output, 
                api_name="get_telemetry"
            )

        # Movement Controls
        with gr.Row():
            distance_input = gr.Number(label="Forward Distance (meters)", value=1.0)
            forward_btn = gr.Button("Move Forward", variant="primary")
            # MCP EXPOSURE: api_name="move_forward"
            forward_btn.click(
                fn=move_forward_sync,
                inputs=[distance_input],
                outputs=robot_output,
                api_name="move_forward" 
            )
        
        with gr.Row():
            left_btn = gr.Button("Turn Left (90°)")
            right_btn = gr.Button("Turn Right (90°)")
            
            # MCP EXPOSURE: api_name="turn_left"
            left_btn.click(
                fn=turn_left_sync,
                inputs=[],
                outputs=robot_output,
                api_name="turn_left" 
            )
            # MCP EXPOSURE: api_name="turn_right"
            right_btn.click(
                fn=turn_right_sync,
                inputs=[],
                outputs=robot_output,
                api_name="turn_right" 
            )


# --- 4. STARTING THE MCP SERVER (Crucial) ---

if __name__ == "__main__":
    print("Starting Gradio/MCP server...")
    # The mcp_server=True option enables the exposure of API endpoints defined by api_name.
    demo.launch(
        mcp_server=True, 
        share=False,
        server_name="localhost",
        server_port=7860
    )
