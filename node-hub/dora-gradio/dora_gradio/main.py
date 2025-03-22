"""TODO: Add docstring."""

import logging
import os
import signal
import time

import cv2
import gradio as gr
import librosa
import numpy as np
import pyarrow as pa
from dora import Node
from fastrtc import Stream

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class DoraGradioUI:
    """TODO: Add docstring."""

    def __init__(self):
        """TODO: Add docstring."""
        self.node = Node()
        self.theme = gr.themes.Soft(primary_hue="blue", secondary_hue="indigo")
        self.sample_rate = 16000
        self.max_duration = 0.1
        self.buffer = []
        self.start_recording_time = time.time()

    def handle_text_input(self, message, history):
        """TODO: Add docstring."""
        if message:
            self.node.send_output("text", pa.array([message]))
            history = history or []
            history.append([message, None])
        return "", history

    def handle_audio_stream(self, stream, new_chunk, text_state):
        """TODO: Add docstring."""
        if new_chunk is not None:
            try:
                sr, y = new_chunk

                # Convert to mono and normalize like microphone node
                if y.ndim > 1:
                    y = y[:, 0]
                y = y.astype(np.float32)
                if np.abs(y).max() > 1.0:
                    y = y / 32768.0  # For int16 input

                # Resample to 16kHz if needed
                if sr != self.sample_rate:
                    y = librosa.resample(y, orig_sr=sr, target_sr=self.sample_rate)

                self.buffer.extend(y)
                current_time = time.time()
                if current_time - self.start_recording_time > self.max_duration:
                    audio_data = np.array(self.buffer, dtype=np.float32).ravel()
                    audio_data = np.clip(audio_data, -1.0, 1.0)
                    self.node.send_output("audio", pa.array(audio_data), {
                        "sample_rate": self.sample_rate,
                        "channels": 1,
                        "timestamp": int(time.time() * 1_000_000_000),
                    })
                    self.buffer = []
                    self.start_recording_time = current_time
                return stream, "", "✓ Streaming audio..."
            except Exception as e:
                logger.error(f"Error processing audio: {e}")
                return stream, text_state, f"❌ Error: {e!s}"
        return None, text_state, "Waiting for audio..."

    def handle_video_stream(self, frame):
        """Handle incoming video frames."""
        if frame is not None:
            try:
                # Convert frame to proper format
                if frame.shape[1] != 640 or frame.shape[0] != 480:
                    frame = cv2.resize(frame, (640, 480))
                timestamp = int(time.time() * 1_000_000_000)
                self.node.send_output("image", pa.array(frame.ravel()), {
                    "encoding": "bgr8",
                    "width": frame.shape[1],
                    "height": frame.shape[0],
                    "timestamp": timestamp,
                    "_time": timestamp,
                })

                return frame
            except Exception as e:
                logger.error(f"Error in video stream: {e}")
                return np.zeros((480, 640, 3), dtype=np.uint8)
        return None

    def create_interface(self):
        """TODO: Add docstring."""
        with gr.Blocks(theme=self.theme, title="Dora Input Interface") as interface:
            gr.Markdown("## Dora Input Interface")
            with gr.Tab("Camera"):

                video_feed = Stream(
                    handler=self.handle_video_stream,
                    modality="video",
                    mode="send-receive",
                    rtc_configuration={
                        "iceServers": [
                            {"urls": ["stun:stun.l.google.com:19302"]},
                        ],
                    },
                )
            with gr.Tab("Audio and Text Input"):
                with gr.Group():
                    gr.Markdown("### Audio Input")
                    audio_state = gr.State(None)
                    text_state = gr.State("")
                    audio_input = gr.Audio(
                        sources=["microphone"],
                        streaming=True,
                        type="numpy",
                    )
                    audio_status = gr.Markdown("Status: Ready")

                with gr.Group():
                    gr.Markdown("### Text Input")
                    chatbot = gr.Chatbot(show_label=False, height=200)
                    with gr.Row():
                        text_input = gr.Textbox(
                            placeholder="Type your message here...",
                            show_label=False,
                            scale=4,
                        )
                        text_send = gr.Button("Send Text", scale=1)

            with gr.Row():
                stop_button = gr.Button("Stop Server", variant="stop", scale=0.5)

            # Event handlers
            text_input.submit(self.handle_text_input, [text_input, chatbot], [text_input, chatbot])
            text_send.click(self.handle_text_input, [text_input, chatbot], [text_input, chatbot])
            audio_input.stream(self.handle_audio_stream, [audio_state, audio_input, text_state], [audio_state, text_state, audio_status])
            stop_button.click(self.kill_server)

        return interface

    def launch(self):
        """TODO: Add docstring."""
        try:
            # Kill existing process on port 7860 if any
            # import subprocess
            # subprocess.run('lsof -ti :7860 | xargs kill -9', shell=True, stderr=subprocess.DEVNULL)
            interface = self.create_interface()

            def cleanup(signum, frame):
                interface.close()
                os._exit(0)

            signal.signal(signal.SIGINT, cleanup)
            signal.signal(signal.SIGTERM, cleanup)
            interface.launch(server_name="0.0.0.0", server_port=7860, quiet=False)

        except Exception as e:
            logger.error(f"Error launching UI: {e}")
            raise e

    def kill_server(self):
        """Kill the Gradio server and exit."""
        logger.info("Manually stopping server...")
        os._exit(0)

def main():
    """TODO: Add docstring."""
    ui = DoraGradioUI()
    ui.launch()

if __name__ == "__main__":
    main()
