import gradio as gr
import numpy as np
import torch
import logging
import pyarrow as pa
from dora import Node
from transformers import pipeline
import subprocess
import signal
import os

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class DoraGradioUI:
    def __init__(self):
        self.node = Node()
        self.theme = gr.themes.Soft(primary_hue="blue", secondary_hue="indigo")
        self.transcriber = pipeline(
            "automatic-speech-recognition", 
            model="openai/whisper-base.en",
            device='cuda' if torch.cuda.is_available() else 'cpu'
        )

    def handle_text_input(self, message, history):
        if message:
            self.node.send_output("text_input", pa.array([message]))
            history = history or []
            history.append([message, None])
        return "", history

    def handle_audio_stream(self, stream, new_chunk, text_state):
        if new_chunk is None and stream is not None:
            return None, "", "✓ Recording stopped"
        if new_chunk is not None:
            try:
                sr, y = new_chunk
                if y.ndim > 1:
                    y = y.mean(axis=1)
                y = y.astype(np.float32)
                y /= np.max(np.abs(y)) if np.max(np.abs(y)) > 0 else 1

                stream = np.concatenate([stream, y]) if stream is not None else y
                text = self.transcriber({"sampling_rate": sr, "raw": stream})["text"]

                if text and text.strip() not in [".", "so", "The", "THANK YOU"]:
                    return stream, text, f"✓ Transcribed: {text}"
                return stream, text_state, "✓ Listening..."

            except Exception as e:
                logger.error(f"Error processing audio: {e}")
                return stream, text_state, f"❌ Error: {str(e)}"
        return None, text_state, "Waiting for audio..."
    
    def handle_send_click(self, text_state):
        if text_state and text_state.strip():
            self.node.send_output("transcribed_text", pa.array([text_state]), {"language": "english"})
            return None, "", f"✓ Sent: {text_state[:30]}..."
        return None, text_state, "No text to send"
    
    def create_interface(self):
        with gr.Blocks(theme=self.theme, title="Dora Input Interface") as interface:
            gr.Markdown("## Dora Input Interface")
            
            with gr.Row():
                with gr.Group():
                    gr.Markdown("### Voice Input")
                    audio_state = gr.State(None)
                    text_state = gr.State("")
                    audio_input = gr.Audio(sources=["microphone"], streaming=True, type="numpy")
                    audio_status = gr.Markdown("Status: Ready")
                    with gr.Row():
                        send_button = gr.Button("Send to Dora", variant="primary")
                        clear_button = gr.Button("Clear", variant="secondary")

            with gr.Group():
                gr.Markdown("### Text Input")
                chatbot = gr.Chatbot(show_label=False)
                text_input = gr.Textbox(placeholder="Type your message here...", show_label=False)

            with gr.Row():
                stop_button = gr.Button("Stop Server", variant="stop", scale=0.5)

            # Existing event handlers
            text_input.submit(self.handle_text_input, [text_input, chatbot], [text_input, chatbot])
            audio_input.stream(self.handle_audio_stream, [audio_state, audio_input, text_state],
                             [audio_state, text_state, audio_status])
            send_button.click(self.handle_send_click, [text_state],
                            [audio_state, text_state, audio_status])
            clear_button.click(lambda: (None, "", "✓ Cleared"), None,
                             [audio_state, text_state, audio_status])

            stop_button.click(self.kill_server)

        return interface

    def launch(self):
        try:
            # Kill existing process on port 7860 if any
            subprocess.run('lsof -ti :7860 | xargs kill -9', shell=True, stderr=subprocess.DEVNULL)
            
            interface = self.create_interface()
            
            def cleanup(signum, frame):
                interface.close()
                os._exit(0)
                
            signal.signal(signal.SIGINT, cleanup)
            signal.signal(signal.SIGTERM, cleanup)
            
            interface.launch(server_name="0.0.0.0", server_port=7860, quiet=True)
            logger.info("Gradio UI running at: http://localhost:7860")
            
            while True:
                import time
                time.sleep(0.1)
                
        except Exception as e:
            logger.error(f"Error launching UI: {e}")
            raise e


    def kill_server(self):
        """Kill the Gradio server and exit"""
        logger.info("Manually stopping server...")
        os._exit(0)

def main():
    ui = DoraGradioUI()
    ui.launch()

if __name__ == "__main__":
    main()
