#!/usr/bin/env python3
"""
Dora Qwen3-8B LLM Node.

Optimized for both MLX (Apple Silicon) and GGUF (CPU/GPU) inference.
"""

import os
import sys
import gc
import re
import json
import time
import platform
from typing import Dict, List, Optional, Union

import pyarrow as pa
from dora import Node

# Check platform for MLX support
IS_APPLE_SILICON = platform.system() == "Darwin" and platform.machine() == "arm64"

# Handle USE_MLX environment variable
use_mlx_env = os.getenv("USE_MLX", "auto").lower()
if use_mlx_env == "auto":
    USE_MLX = IS_APPLE_SILICON
elif use_mlx_env == "true":
    USE_MLX = True
else:
    USE_MLX = False

# Try to import MLX if needed
MLX_AVAILABLE = False
if USE_MLX and IS_APPLE_SILICON:
    try:
        import mlx
        import mlx.core as mx
        from mlx_lm import load, generate
        MLX_AVAILABLE = True
        # MLX imported successfully - will log after node is ready
    except ImportError as e:
        # MLX import failed - will log after node is ready
        USE_MLX = False

# Import GGUF backend if not using MLX
if not USE_MLX:
    try:
        from llama_cpp import Llama
    except ImportError:
        print("Error: llama-cpp-python not installed!")
        print("Install with: pip install llama-cpp-python")
        sys.exit(1)


def send_log(node, level, message):
    """Send log message through log output channel.
    
    Args:
        node: Dora node instance
        level: Log level (DEBUG, INFO, WARNING, ERROR)
        message: Log message
    """
    # Define log level hierarchy
    LOG_LEVELS = {
        "DEBUG": 10,
        "INFO": 20,
        "WARNING": 30,
        "ERROR": 40
    }
    
    # Get configured log level from environment
    config_level = os.getenv("LOG_LEVEL", "INFO").upper()
    
    # Check if message should be logged
    if LOG_LEVELS.get(level, 0) < LOG_LEVELS.get(config_level, 20):
        return  # Skip messages below configured level
    
    # Format message with level prefix
    formatted_message = f"[{level}] {message}"
    
    log_data = {
        "node": "qwen3-llm",
        "level": level,
        "message": formatted_message,
        "timestamp": time.time()
    }
    node.send_output("log", pa.array([json.dumps(log_data)]))


class Qwen3LLMNode:
    """Qwen3-8B LLM node for Dora dataflows."""

    # Qwen3 chat template tokens
    SYSTEM_START = "<|im_start|>system\n"
    SYSTEM_END = "<|im_end|>\n"
    USER_START = "<|im_start|>user\n"
    USER_END = "<|im_end|>\n"
    ASSISTANT_START = "<|im_start|>assistant\n"
    ASSISTANT_END = "<|im_end|>"

    def __init__(self):
        """Initialize the LLM node."""
        self.node = None  # Will be initialized in run()
        self.model = None
        self.tokenizer = None  # For MLX
        self.use_mlx = USE_MLX and MLX_AVAILABLE
        self.chat_sessions = {}
        self.config = self.load_config()

        # Will be initialized after node is available
        self.model_loaded = False
        
    def initialize_model(self):
        """Initialize model after node is available."""
        if not self.model_loaded:
            send_log(self.node, "INFO", f"Initializing Qwen3-8B ({'MLX' if self.use_mlx else 'GGUF'})...")
            self.load_model()
            send_log(self.node, "INFO", "Ready for chat!")
            send_log(self.node, "INFO", f"Log level: {self.config['log_level']}")
            self.model_loaded = True

    def load_config(self) -> Dict:
        """Load configuration from environment variables."""
        config = {
            # GGUF settings
            "gguf_model": os.getenv("GGUF_MODEL", "Qwen/Qwen3-8B-GGUF"),
            "gguf_model_file": os.getenv("GGUF_MODEL_FILE", "Qwen3-8B-Q4_K_M.gguf"),
            "n_gpu_layers": int(os.getenv("N_GPU_LAYERS", "0")),
            "n_threads": int(os.getenv("N_THREADS", "4")),
            "context_size": int(os.getenv("CONTEXT_SIZE", "4096")),

            # Generation settings
            "max_tokens": int(os.getenv("MAX_TOKENS", "2048")),
            "temperature": float(os.getenv("TEMPERATURE", "0.7")),
            "enable_thinking": os.getenv("ENABLE_THINKING", "false").lower() == "true",

            # System prompt
            "system_prompt": os.getenv("SYSTEM_PROMPT", "You are a helpful AI assistant."),
            
            # Logging
            "log_level": os.getenv("LOG_LEVEL", "INFO").upper(),
        }

        # MLX-specific config
        if self.use_mlx:
            config["mlx_model"] = os.getenv("MLX_MODEL", "Qwen/Qwen3-8B-MLX-4bit")
            config["mlx_max_tokens"] = int(os.getenv("MLX_MAX_TOKENS", "2048"))
            config["mlx_temp"] = float(os.getenv("MLX_TEMPERATURE", "0.7"))

        return config

    def load_model(self):
        """Load the model - MLX or GGUF depending on platform."""
        if self.use_mlx:
            self.load_model_mlx()
        else:
            self.load_model_gguf()

    def load_model_mlx(self):
        """Load Qwen3 model using MLX for Apple Silicon."""
        try:
            model_id = self.config["mlx_model"]
            send_log(self.node, "INFO", f"Loading MLX model: {model_id}")
            send_log(self.node, "INFO", "This may take a minute on first run...")

            # Load model and tokenizer
            self.model, self.tokenizer = load(model_id)
            send_log(self.node, "INFO", f"MLX model loaded: {model_id}")
            send_log(self.node, "INFO", "Using Metal GPU acceleration")

        except Exception as e:
            send_log(self.node, "ERROR", f"Error loading MLX model: {e}")
            send_log(self.node, "WARNING", "Falling back to GGUF...")
            self.use_mlx = False
            self.load_model_gguf()

    def load_model_gguf(self):
        """Load Qwen3 model in GGUF format."""
        local_path = f"./models/{self.config['gguf_model_file']}"

        try:
            if os.path.exists(local_path):
                send_log(self.node, "INFO", f"Loading local GGUF model: {local_path}")
                self.model = Llama(
                    model_path=local_path,
                    n_gpu_layers=self.config["n_gpu_layers"],
                    n_threads=self.config["n_threads"],
                    n_ctx=self.config["context_size"],
                    verbose=False,
                    chat_format="chatml",
                )
            else:
                send_log(self.node, "INFO", "Downloading GGUF model...")
                self.model = Llama.from_pretrained(
                    repo_id=self.config["gguf_model"],
                    filename=self.config["gguf_model_file"],
                    n_gpu_layers=self.config["n_gpu_layers"],
                    n_threads=self.config["n_threads"],
                    n_ctx=self.config["context_size"],
                    verbose=False,
                    chat_format="chatml",
                )
            send_log(self.node, "INFO", "GGUF model loaded!")
        except Exception as e:
            send_log(self.node, "ERROR", f"Error loading GGUF model: {e}")
            # Try a fallback model
            try:
                send_log(self.node, "WARNING", "Trying fallback model...")
                self.model = Llama.from_pretrained(
                    repo_id="Qwen/Qwen2.5-0.5B-Instruct-GGUF",
                    filename="*q4_0.gguf",
                    n_gpu_layers=0,
                    n_threads=self.config["n_threads"],
                    n_ctx=2048,
                    verbose=False,
                    chat_format="chatml",
                )
                send_log(self.node, "INFO", "Fallback model loaded")
            except Exception as e2:
                send_log(self.node, "ERROR", f"Failed to load fallback: {e2}")
                sys.exit(1)

    def generate_response(self, text: str, session_id: str) -> str:
        """Generate a response using MLX or GGUF."""
        if self.use_mlx:
            return self.generate_response_mlx(text, session_id)
        else:
            return self.generate_response_gguf(text, session_id)

    def generate_response_mlx(self, text: str, session_id: str) -> str:
        """Generate response using MLX."""
        try:
            # Get or create session
            if session_id not in self.chat_sessions:
                self.chat_sessions[session_id] = []

            history = self.chat_sessions[session_id]

            # Prepare prompt with thinking control
            system_prompt = self.config["system_prompt"]
            if not self.config["enable_thinking"]:
                system_prompt += " Provide direct answers without showing your thinking process."
                text = text + " /no_think"

            # Format messages for MLX
            messages = [{"role": "system", "content": system_prompt}]
            messages.extend(history)
            messages.append({"role": "user", "content": text})

            # Apply chat template
            prompt = self.tokenizer.apply_chat_template(
                messages,
                tokenize=False,
                add_generation_prompt=True
            )

            send_log(self.node, "DEBUG", "Generating with MLX...")

            # Generate response
            response = generate(
                self.model,
                self.tokenizer,
                prompt,
                max_tokens=self.config["mlx_max_tokens"],
                verbose=False
            )

            # Clean response
            if prompt in response:
                response = response.replace(prompt, "").strip()

            # Handle thinking tags if disabled
            if not self.config["enable_thinking"]:
                # Remove complete thinking tags
                response = re.sub(r'<think>.*?</think>', '', response, flags=re.DOTALL).strip()
                # Remove incomplete opening tag at the beginning
                if response.startswith('<think>'):
                    response = response[7:].strip()  # Remove '<think>' (7 chars)

            # Update history
            history.append({"role": "user", "content": text})
            history.append({"role": "assistant", "content": response})

            # Keep history manageable
            if len(history) > 10:
                history = history[-10:]
                self.chat_sessions[session_id] = history

            return response

        except Exception as e:
            send_log(self.node, "ERROR", f"Error with MLX generation: {e}")
            return f"Error: {str(e)}"

    def generate_response_gguf(self, text: str, session_id: str) -> str:
        """Generate response using GGUF."""
        try:
            # Get or create session
            if session_id not in self.chat_sessions:
                self.chat_sessions[session_id] = []

            history = self.chat_sessions[session_id]

            # Prepare system prompt
            system_prompt = self.config["system_prompt"]
            if not self.config["enable_thinking"]:
                system_prompt += " Provide direct answers without showing your thinking process."

            # Add user message
            history.append({"role": "user", "content": text})

            # Keep history manageable
            if len(history) > 10:
                history = history[-10:]
                self.chat_sessions[session_id] = history

            # Format prompt manually using Qwen3 template
            prompt = f"{self.SYSTEM_START}{system_prompt}{self.SYSTEM_END}"
            for msg in history:
                if msg["role"] == "user":
                    prompt += f"{self.USER_START}{msg['content']}{self.USER_END}"
                elif msg["role"] == "assistant":
                    prompt += f"{self.ASSISTANT_START}{msg['content']}{self.ASSISTANT_END}"

            # Add assistant start for generation
            prompt += self.ASSISTANT_START

            send_log(self.node, "DEBUG", "Generating response...")

            # Generate response
            response = self.model(
                prompt,
                max_tokens=self.config["max_tokens"],
                temperature=self.config["temperature"],
                stop=[self.ASSISTANT_END, "<|im_end|>", "<|endoftext|>"],
                echo=False,
            )

            content = response["choices"][0]["text"]

            # Handle thinking tags if disabled
            if not self.config["enable_thinking"]:
                # Remove complete thinking tags
                content = re.sub(r'<think>.*?</think>', '', content, flags=re.DOTALL).strip()
                # Remove incomplete opening tag at the beginning
                if content.startswith('<think>'):
                    content = content[7:].strip()  # Remove '<think>' (7 chars)

            # Add to history
            history.append({"role": "assistant", "content": content})

            return content

        except Exception as e:
            send_log(self.node, "ERROR", f"Error generating response: {e}")
            return f"Error: {str(e)}"

    def run(self):
        """Main event loop for the node."""
        # Connecting to dataflow

        # Initialize Node here where dataflow context is available
        self.node = Node()

        # Initialize model after node is available
        self.initialize_model()
        send_log(self.node, "INFO", "Node started, waiting for input...")

        try:
            for event in self.node:
                if event["type"] == "INPUT":
                    input_id = event["id"]

                    # Skip timer ticks
                    if input_id == "tick":
                        continue

                    metadata = event.get("metadata", {})
                    session_id = metadata.get("session_id", "default")

                    # Handle text input
                    if input_id == "text":
                        try:
                            user_text = event["value"][0].as_py()
                            send_log(self.node, "DEBUG", f"Input: {user_text[:100]}...")

                            response = self.generate_response(user_text, session_id)

                            # Send response
                            self.node.send_output(
                                output_id="text",
                                data=pa.array([response]),
                                metadata={"session_id": session_id}
                            )

                            send_log(self.node, "INFO", f"Response Generated ({len(response)} chars)")

                        except Exception as e:
                            send_log(self.node, "ERROR", f"Error processing text: {e}")
                            self.node.send_output(
                                output_id="response",
                                data=pa.array([f"Error: {str(e)}"]),
                                metadata={"session_id": session_id}
                            )

                    # Handle control commands
                    elif input_id == "text_to_audio":
                        # Handle text_to_audio input (for openai-realtime compatibility)
                        try:
                            user_text = event["value"][0].as_py()
                            send_log(self.node, "DEBUG", f"Text-to-audio input: {user_text[:100]}...")

                            response = self.generate_response(user_text, session_id)

                            # Send response as text output (will be converted to audio by TTS)
                            self.node.send_output(
                                output_id="text",
                                data=pa.array([response]),
                                metadata={"session_id": session_id}
                            )

                            send_log(self.node, "INFO", f"Text-to-audio response sent ({len(response)} chars)")

                        except Exception as e:
                            send_log(self.node, "ERROR", f"Error processing text_to_audio: {e}")
                            import traceback
                            traceback.print_exc()

                    elif input_id == "control":
                        command = event["value"][0].as_py()
                        send_log(self.node, "DEBUG", f"Control: {command}")

                        if command == "exit":
                            if session_id in self.chat_sessions:
                                del self.chat_sessions[session_id]
                            send_log(self.node, "INFO", f"Session closed: {session_id}")
                        elif command == "reset":
                            self.chat_sessions[session_id] = []
                            send_log(self.node, "INFO", f"Session reset: {session_id}")
                        elif command == "ready":
                            send_log(self.node, "DEBUG", f"Client ready: {session_id}")
                            self.node.send_output(
                                output_id="status",
                                data=pa.array(["ready"]),
                                metadata={"session_id": session_id}
                            )

        except KeyboardInterrupt:
            send_log(self.node, "INFO", "Interrupted")
        except Exception as e:
            send_log(self.node, "ERROR", f"Fatal error: {e}")
        finally:
            send_log(self.node, "INFO", "Shutting down...")


def main():
    """Main entry point for the dora-qwen3 node."""
    try:
        llm = Qwen3LLMNode()
        llm.run()
    except Exception as e:
        print(f"Fatal error in main: {e}")  # Keep this for critical errors
        import traceback
        traceback.print_exc()
        raise


if __name__ == "__main__":
    main()
