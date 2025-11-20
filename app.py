import gradio as gr
import asyncio
from huggingface_hub import InferenceClient

# Importa l'istanza del robot e la sua logica da mcp_server.py
from mcp_server import robot_instance 

# --- 1. FUNZIONI WRAPPER SINCRONE PER IL ROBOT (Necessarie per Gradio) ---

def get_telemetry_sync() -> str:
    """Restituisce lo stato del robot formattato per l'interfaccia."""
    state = robot_instance.get_state()
    return f"Telemetria: X={state['x']}, Y={state['y']}, Dir={state['heading']}°, Distanza Percorsa={state['distance_traveled']:.2f}"

def move_forward_sync(distance: float) -> str:
    """Esegue move_forward asincrono e restituisce lo stato aggiornato."""
    if distance <= 0:
        return "Errore: la distanza deve essere un numero positivo."
    
    try:
        # Usa asyncio.run per chiamare la funzione asincrona
        result = asyncio.run(robot_instance.move_forward(distance))
        state = result['current_state']
        return f"Avanzamento di {distance} metri completato.\nStato: X={state['x']}, Y={state['y']}, Dir={state['heading']}°"
    except ValueError as e:
        return f"Errore: {e}"

def turn_left_sync() -> str:
    """Esegue turn_left asincrono e restituisce lo stato aggiornato."""
    result = asyncio.run(robot_instance.turn_left())
    state = result['current_state']
    return f"Giro a SINISTRA completato.\nStato: X={state['x']}, Y={state['y']}, Dir={state['heading']}°"

def turn_right_sync() -> str:
    """Esegue turn_right asincrono e restituisce lo stato aggiornato."""
    result = asyncio.run(robot_instance.turn_right())
    state = result['current_state']
    return f"Giro a DESTRA completato.\nStato: X={state['x']}, Y={state['y']}, Dir={state['heading']}°"


# --- 2. FUNZIONE E INTERFACCIA CHATBOT ORIGINALE ---

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
    Funzione originale per l'interazione con l'LLM tramite Hugging Face Inference API.
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

# --- 3. STRUTTURA BLOCKS CON DUE TAB (Chatbot e Robot) ---

with gr.Blocks(title="Robot Control and Chatbot MCP") as demo:
    with gr.Sidebar():
        gr.LoginButton()
    
    # PRIMA TAB: La Chatbot originale
    with gr.Tab("Chatbot"): 
        chatbot.render()
        
    # SECONDA TAB: L'interfaccia di controllo del Robot/MCP Tools
    with gr.Tab("Controllo Robot MCP"): 
        gr.Markdown("## 🤖 Interfaccia di Controllo e Telemetria del Robot")
        robot_output = gr.Textbox(label="Risultato Ultima Azione / Telemetria")

        # Telemetria
        with gr.Row():
            telemetry_btn = gr.Button("Aggiorna Telemetria", variant="secondary")
            # ESPOSIZIONE MCP: api_name="get_telemetry"
            telemetry_btn.click(
                fn=get_telemetry_sync, 
                inputs=[], 
                outputs=robot_output, 
                api_name="get_telemetry"
            )

        # Controlli di Movimento
        with gr.Row():
            distance_input = gr.Number(label="Distanza Avanti (metri)", value=1.0)
            forward_btn = gr.Button("Vai Avanti", variant="primary")
            # ESPOSIZIONE MCP: api_name="move_forward"
            forward_btn.click(
                fn=move_forward_sync,
                inputs=[distance_input],
                outputs=robot_output,
                api_name="move_forward" 
            )
        
        with gr.Row():
            left_btn = gr.Button("Gira a Sinistra (90°)")
            right_btn = gr.Button("Gira a Destra (90°)")
            
            # ESPOSIZIONE MCP: api_name="turn_left"
            left_btn.click(
                fn=turn_left_sync,
                inputs=[],
                outputs=robot_output,
                api_name="turn_left" 
            )
            # ESPOSIZIONE MCP: api_name="turn_right"
            right_btn.click(
                fn=turn_right_sync,
                inputs=[],
                outputs=robot_output,
                api_name="turn_right" 
            )


# --- 4. AVVIO DEL SERVER MCP (Cruciale) ---

if __name__ == "__main__":
    print("Avvio del server Gradio/MCP...")
    # L'opzione mcp_server=True abilita l'esposizione degli endpoint API definiti da api_name.
    demo.launch(
        mcp_server=True, 
        share=False,
        server_name="localhost",
        server_port=7860
    )