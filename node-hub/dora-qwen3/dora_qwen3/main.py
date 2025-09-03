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
        from mlx_lm import load, generate, stream_generate
        from mlx_lm.models.cache import make_prompt_cache
        from mlx_lm.sample_utils import make_sampler, make_logits_processors
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
            if self.use_mlx:
                model_name = self.config.get("mlx_model", "Unknown MLX model")
            else:
                model_name = self.config.get("gguf_model", "Unknown GGUF model")
            send_log(self.node, "INFO", f"Initializing {model_name} ({'MLX' if self.use_mlx else 'GGUF'})...")
            self.load_model()
            send_log(self.node, "INFO", "Ready for chat!")
            send_log(self.node, "INFO", f"Log level: {self.config['log_level']}")
            
            # Log history management strategy
            strategy = self.config["history_strategy"]
            if strategy == "fixed":
                send_log(self.node, "INFO", 
                        f"History: Fixed strategy - keeping last {self.config['max_history_exchanges']} exchanges")
            elif strategy == "token_based":
                send_log(self.node, "INFO", 
                        f"History: Token-based strategy - max {self.config['max_history_tokens']} tokens")
            else:
                send_log(self.node, "INFO", f"History: {strategy} strategy")
            self.model_loaded = True

    def format_gemma_prompt(self, messages: List[Dict]) -> str:
        """Format messages using Gemma's chat template.
        
        Gemma uses:
        <bos><start_of_turn>user\n{user_message}<end_of_turn>
        <start_of_turn>model\n{model_message}<end_of_turn>
        """
        prompt = "<bos>"
        
        for msg in messages:
            role = msg["role"]
            content = msg["content"]
            
            if role == "system":
                # Gemma doesn't have explicit system role, include as first user message
                prompt += f"<start_of_turn>user\nSystem: {content}<end_of_turn>\n"
            elif role == "user":
                prompt += f"<start_of_turn>user\n{content}<end_of_turn>\n"
            elif role == "assistant":
                prompt += f"<start_of_turn>model\n{content}<end_of_turn>\n"
        
        # Add the model turn start for generation
        prompt += "<start_of_turn>model\n"
        
        return prompt

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
            
            # Chat history settings
            "max_history_exchanges": int(os.getenv("MAX_HISTORY_EXCHANGES", "10")),
            "history_strategy": os.getenv("HISTORY_STRATEGY", "fixed"),  # fixed, token_based, or sliding_window
            "max_history_tokens": int(os.getenv("MAX_HISTORY_TOKENS", "2000")),
            
            # Logging
            "log_level": os.getenv("LOG_LEVEL", "INFO").upper(),
        }

        # MLX-specific config
        if self.use_mlx:
            # Support both Qwen3 and Gemma models
            mlx_model = os.getenv("MLX_MODEL", "Qwen/Qwen3-8B-MLX-4bit")
            config["mlx_model"] = mlx_model
            config["mlx_max_tokens"] = int(os.getenv("MLX_MAX_TOKENS", "2048"))
            config["mlx_temp"] = float(os.getenv("MLX_TEMPERATURE", "0.7"))
            
            # These parameters work for all MLX models
            config["top_p"] = float(os.getenv("TOP_P", "0.0"))  # 0 means disabled
            config["repetition_penalty"] = float(os.getenv("REPETITION_PENALTY", "1.0"))  # 1.0 means disabled
            
            # Detect model type from model ID
            if "gemma" in mlx_model.lower():
                config["model_type"] = "gemma"
            elif "glm" in mlx_model.lower():
                config["model_type"] = "glm"
            else:
                config["model_type"] = "qwen"

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
            
            # Check if model needs downloading
            from huggingface_hub import snapshot_download
            import os
            cache_dir = os.path.expanduser("~/.cache/huggingface/hub")
            model_cache = os.path.join(cache_dir, f"models--{model_id.replace('/', '--')}")
            
            if not os.path.exists(model_cache):
                send_log(self.node, "INFO", f"Model not cached, downloading {model_id}...")
                send_log(self.node, "INFO", "This will take several minutes for first download")
                # Download with progress
                snapshot_download(model_id, cache_dir=cache_dir)
                send_log(self.node, "INFO", "Download complete!")
            else:
                send_log(self.node, "INFO", f"Using cached model from {model_cache}")

            # Load model and tokenizer
            send_log(self.node, "INFO", "Loading model into memory...")
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

    def estimate_tokens(self, text: str) -> int:
        """Estimate token count for text using actual tokenizer."""
        if self.tokenizer:
            try:
                tokens = self.tokenizer.encode(text)
                return len(tokens)
            except:
                pass
        # Fallback to rough estimation
        return int(len(text.split()) * 1.3)

    def manage_history(self, history: List[Dict], new_user_msg: str = None) -> List[Dict]:
        """
        Intelligently manage conversation history based on configured strategy.
        
        Args:
            history: Current conversation history
            new_user_msg: Optional new user message to estimate tokens for
            
        Returns:
            Managed history list that fits within constraints
        """
        strategy = self.config["history_strategy"]
        max_exchanges = self.config["max_history_exchanges"]
        
        if strategy == "fixed":
            # Simple fixed-length history (current behavior)
            if len(history) > max_exchanges * 2:  # *2 because each exchange has user + assistant
                return history[-(max_exchanges * 2):]
                
        elif strategy == "token_based":
            # Estimate tokens and trim oldest messages if needed
            max_tokens = self.config["max_history_tokens"]
            estimated_tokens = 0
            kept_history = []
            
            # Estimate tokens for new message if provided
            if new_user_msg:
                estimated_tokens += self.estimate_tokens(new_user_msg)
            
            # Keep messages from newest to oldest until token limit
            for msg in reversed(history):
                msg_tokens = self.estimate_tokens(msg["content"])
                if estimated_tokens + msg_tokens < max_tokens:
                    kept_history.insert(0, msg)
                    estimated_tokens += msg_tokens
                else:
                    break
                    
            # Ensure we keep at least one exchange for context
            if len(kept_history) < 2 and len(history) >= 2:
                kept_history = history[-2:]
                
            return kept_history
            
        elif strategy == "sliding_window":
            # Keep recent exchanges but summarize older ones (future enhancement)
            # For now, fall back to fixed strategy
            if len(history) > max_exchanges * 2:
                return history[-(max_exchanges * 2):]
                
        return history

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

            # Manage history BEFORE using it to ensure we stay within limits
            history = self.manage_history(self.chat_sessions[session_id], text)

            # Prepare prompt with thinking control
            system_prompt = self.config["system_prompt"]
            model_type = self.config.get("model_type", "qwen")
            
            # Handle thinking mode for different models
            if not self.config["enable_thinking"]:
                if model_type == "glm":
                    # GLM uses /nothink suffix in user message
                    text = text + "/nothink"
                elif model_type == "gemma":
                    # Gemma uses system prompt instruction
                    system_prompt += " Provide direct answers without showing your thinking process."
                else:
                    # Qwen uses /no_think suffix
                    text = text + " /no_think"
                    system_prompt += " Provide direct answers without showing your thinking process."

            # Format messages for MLX with managed history
            messages = [{"role": "system", "content": system_prompt}]
            messages.extend(history)
            messages.append({"role": "user", "content": text})

            # Apply chat template based on model type
            if self.config.get("model_type") == "gemma":
                # Use Gemma's specific chat format
                prompt = self.format_gemma_prompt(messages)
            elif self.config.get("model_type") == "glm":
                # GLM models have their own chat template
                try:
                    prompt = self.tokenizer.apply_chat_template(
                        messages,
                        tokenize=False,
                        add_generation_prompt=True,
                        enable_thinking=self.config["enable_thinking"]
                    )
                    send_log(self.node, "DEBUG", f"GLM chat template applied (thinking={'enabled' if self.config['enable_thinking'] else 'disabled'})")
                except Exception as e:
                    send_log(self.node, "DEBUG", f"GLM chat template error: {e}, using fallback")
                    # Fallback format for GLM
                    prompt = "[gMASK]<sop>"
                    for msg in messages:
                        if msg["role"] == "system":
                            prompt += f"<|system|>\n{msg['content']}"
                        elif msg["role"] == "user":
                            prompt += f"<|user|>\n{msg['content']}"
                        elif msg["role"] == "assistant":
                            prompt += f"<|assistant|>\n{msg['content']}"
                    prompt += "<|assistant|>\n"
            else:
                # Use tokenizer's default chat template (for Qwen)
                prompt = self.tokenizer.apply_chat_template(
                    messages,
                    tokenize=False,
                    add_generation_prompt=True
                )

            send_log(self.node, "DEBUG", "Generating with MLX...")
            
            # Create KV cache for faster generation
            prompt_cache = None
            if len(history) > 0:  # Use cache if there's conversation history
                try:
                    prompt_cache = make_prompt_cache(self.model)
                    send_log(self.node, "DEBUG", f"Using KV cache (history: {len(history)} messages)")
                except Exception as e:
                    send_log(self.node, "DEBUG", f"KV cache not available: {e}")

            # Generate response with optional KV cache
            generate_params = {
                "model": self.model,
                "tokenizer": self.tokenizer,
                "prompt": prompt,
                "max_tokens": self.config["mlx_max_tokens"],
                "verbose": False
            }
            
            # Add sampling parameters using proper MLX sampler/logits_processors
            # Only apply custom sampler for Gemma or when explicitly configured
            if self.config.get("model_type") == "gemma":
                # Gemma models benefit from custom sampling
                temp = self.config.get("mlx_temp", 0.7)
                top_p = self.config.get("top_p", 0.9)
                
                # Create sampler for Gemma
                if temp > 0 or top_p > 0:
                    sampler = make_sampler(temp=temp, top_p=top_p)
                    generate_params["sampler"] = sampler
                
                # Create logits_processors for repetition penalty
                rep_penalty = self.config.get("repetition_penalty", 1.1)
                if rep_penalty != 1.0:
                    logits_processors = make_logits_processors(repetition_penalty=rep_penalty)
                    generate_params["logits_processors"] = logits_processors
            else:
                # For Qwen and other models, only use sampler if explicitly set
                # Check if user explicitly set these (not using defaults)
                if "MLX_TEMPERATURE" in os.environ or "TOP_P" in os.environ:
                    temp = self.config.get("mlx_temp", 0.0)
                    top_p = self.config.get("top_p", 0.0)
                    
                    if temp > 0 or top_p > 0:
                        sampler = make_sampler(temp=temp, top_p=top_p)
                        generate_params["sampler"] = sampler
                
                # Only apply repetition penalty if explicitly set
                if "REPETITION_PENALTY" in os.environ:
                    rep_penalty = self.config.get("repetition_penalty", 1.0)
                    if rep_penalty != 1.0:
                        logits_processors = make_logits_processors(repetition_penalty=rep_penalty)
                        generate_params["logits_processors"] = logits_processors
                
            if prompt_cache is not None:
                generate_params["prompt_cache"] = prompt_cache
                
            response = generate(**generate_params)

            # Clean response
            if prompt in response:
                response = response.replace(prompt, "").strip()
            
            # Clean model-specific end tokens
            if self.config.get("model_type") == "gemma":
                # Remove Gemma's end tokens
                response = response.replace("<end_of_turn>", "").strip()
                response = response.replace("<eos>", "").strip()
            elif self.config.get("model_type") == "glm":
                # Remove GLM's end tokens if any
                response = response.replace("<|endoftext|>", "").strip()
                response = response.replace("<|user|>", "").strip()
                response = response.replace("<|assistant|>", "").strip()
            else:
                # Remove Qwen's end tokens (if any remain)
                response = response.replace("<|im_end|>", "").strip()
                response = response.replace("<|im_start|>", "").strip()

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

            # Manage history based on configured strategy
            history = self.manage_history(history)
            self.chat_sessions[session_id] = history

            return response

        except Exception as e:
            send_log(self.node, "ERROR", f"Error with MLX generation: {e}")
            return f"Error: {str(e)}"
    
    def generate_response_mlx_streaming(self, text: str, session_id: str, metadata: dict):
        """Generate response using MLX with streaming."""
        try:
            # Get or create session
            if session_id not in self.chat_sessions:
                self.chat_sessions[session_id] = []

            # Manage history BEFORE using it to ensure we stay within limits
            history = self.manage_history(self.chat_sessions[session_id], text)

            # Prepare prompt with thinking control
            system_prompt = self.config["system_prompt"]
            model_type = self.config.get("model_type", "qwen")
            
            # Handle thinking mode for different models
            if not self.config["enable_thinking"]:
                if model_type == "glm":
                    # GLM uses /nothink suffix in user message
                    text = text + "/nothink"
                elif model_type == "gemma":
                    # Gemma uses system prompt instruction
                    system_prompt += " Provide direct answers without showing your thinking process."
                else:
                    # Qwen uses /no_think suffix
                    text = text + " /no_think"
                    system_prompt += " Provide direct answers without showing your thinking process."

            # Format messages for MLX with managed history
            messages = [{"role": "system", "content": system_prompt}]
            messages.extend(history)
            messages.append({"role": "user", "content": text})

            # Apply chat template based on model type
            if self.config.get("model_type") == "gemma":
                # Use Gemma's specific chat format
                prompt = self.format_gemma_prompt(messages)
            elif self.config.get("model_type") == "glm":
                # GLM models have their own chat template
                try:
                    prompt = self.tokenizer.apply_chat_template(
                        messages,
                        tokenize=False,
                        add_generation_prompt=True,
                        enable_thinking=self.config["enable_thinking"]
                    )
                    send_log(self.node, "DEBUG", f"GLM chat template applied (thinking={'enabled' if self.config['enable_thinking'] else 'disabled'})")
                except Exception as e:
                    send_log(self.node, "DEBUG", f"GLM chat template error: {e}, using fallback")
                    # Fallback format for GLM
                    prompt = "[gMASK]<sop>"
                    for msg in messages:
                        if msg["role"] == "system":
                            prompt += f"<|system|>\n{msg['content']}"
                        elif msg["role"] == "user":
                            prompt += f"<|user|>\n{msg['content']}"
                        elif msg["role"] == "assistant":
                            prompt += f"<|assistant|>\n{msg['content']}"
                    prompt += "<|assistant|>\n"
            else:
                # Use tokenizer's default chat template (for Qwen)
                prompt = self.tokenizer.apply_chat_template(
                    messages,
                    tokenize=False,
                    add_generation_prompt=True
                )

            send_log(self.node, "DEBUG", "Generating with MLX (streaming)...")
            
            # Create KV cache for faster generation
            prompt_cache = None
            if len(history) > 0:  # Use cache if there's conversation history
                try:
                    prompt_cache = make_prompt_cache(self.model)
                    send_log(self.node, "DEBUG", f"Using KV cache (history: {len(history)} messages)")
                except Exception as e:
                    send_log(self.node, "DEBUG", f"KV cache not available: {e}")

            # Stream response
            accumulated_response = ""
            chunk_buffer = ""
            chunk_size = 20  # Send smaller chunks for faster first response
            segment_index = 0
            first_chunk_sent = False
            
            # Add prompt_cache to stream_generate for KV caching
            stream_params = {
                "model": self.model,
                "tokenizer": self.tokenizer,
                "prompt": prompt,
                "max_tokens": self.config["mlx_max_tokens"]
            }
            
            # Add sampling parameters using proper MLX sampler/logits_processors for streaming
            # Only apply custom sampler for Gemma or when explicitly configured
            if self.config.get("model_type") == "gemma":
                # Gemma models benefit from custom sampling
                temp = self.config.get("mlx_temp", 0.7)
                top_p = self.config.get("top_p", 0.9)
                
                # Create sampler for Gemma
                if temp > 0 or top_p > 0:
                    sampler = make_sampler(temp=temp, top_p=top_p)
                    stream_params["sampler"] = sampler
                
                # Create logits_processors for repetition penalty
                rep_penalty = self.config.get("repetition_penalty", 1.1)
                if rep_penalty != 1.0:
                    logits_processors = make_logits_processors(repetition_penalty=rep_penalty)
                    stream_params["logits_processors"] = logits_processors
            else:
                # For Qwen and other models, only use sampler if explicitly set
                # Check if user explicitly set these (not using defaults)
                if "MLX_TEMPERATURE" in os.environ or "TOP_P" in os.environ:
                    temp = self.config.get("mlx_temp", 0.0)
                    top_p = self.config.get("top_p", 0.0)
                    
                    if temp > 0 or top_p > 0:
                        sampler = make_sampler(temp=temp, top_p=top_p)
                        stream_params["sampler"] = sampler
                
                # Only apply repetition penalty if explicitly set
                if "REPETITION_PENALTY" in os.environ:
                    rep_penalty = self.config.get("repetition_penalty", 1.0)
                    if rep_penalty != 1.0:
                        logits_processors = make_logits_processors(repetition_penalty=rep_penalty)
                        stream_params["logits_processors"] = logits_processors
                
            if prompt_cache is not None:
                stream_params["prompt_cache"] = prompt_cache
            
            for token_response in stream_generate(**stream_params):
                # Extract token text (stream_generate returns GenerationResponse objects)
                token_text = token_response.text if hasattr(token_response, 'text') else str(token_response)
                accumulated_response += token_text
                chunk_buffer += token_text
                
                # Check if we should send a chunk (on punctuation or size)
                should_send = False
                if len(chunk_buffer) >= chunk_size:
                    should_send = True
                elif any(p in chunk_buffer for p in ['。', '！', '？', '.', '!', '?', '，', ',']):
                    # Send on punctuation for more natural breaks
                    should_send = True
                
                if should_send and chunk_buffer.strip():
                    # Clean chunk before sending
                    clean_chunk = chunk_buffer.strip()
                    
                    # Skip if it's part of thinking tags
                    if not self.config["enable_thinking"]:
                        if '<think>' in clean_chunk or '</think>' in clean_chunk:
                            chunk_buffer = ""
                            continue
                    
                    # Send chunk to TTS
                    self.node.send_output(
                        output_id="text",
                        data=pa.array([clean_chunk]),
                        metadata={
                            **metadata,
                            "segment_index": segment_index,
                            "is_streaming": True,
                            "is_final": False
                        }
                    )
                    
                    segment_index += 1
                    chunk_buffer = ""
            
            # Send any remaining buffer
            if chunk_buffer.strip():
                self.node.send_output(
                    output_id="text",
                    data=pa.array([chunk_buffer.strip()]),
                    metadata={
                        **metadata,
                        "segment_index": segment_index,
                        "is_streaming": True,
                        "is_final": True
                    }
                )
            
            # Clean final response
            if prompt in accumulated_response:
                accumulated_response = accumulated_response.replace(prompt, "").strip()
            
            # Handle thinking tags if disabled
            if not self.config["enable_thinking"]:
                accumulated_response = re.sub(r'<think>.*?</think>', '', accumulated_response, flags=re.DOTALL).strip()
                if accumulated_response.startswith('<think>'):
                    accumulated_response = accumulated_response[7:].strip()

            # Add current exchange to session history
            self.chat_sessions[session_id].append({"role": "user", "content": text})
            self.chat_sessions[session_id].append({"role": "assistant", "content": accumulated_response})

            # Manage history after adding new exchange to keep within limits
            self.chat_sessions[session_id] = self.manage_history(self.chat_sessions[session_id])

            return accumulated_response

        except Exception as e:
            send_log(self.node, "ERROR", f"Error with MLX streaming generation: {e}")
            # Send error as final message
            self.node.send_output(
                output_id="text",
                data=pa.array([f"Error: {str(e)}"]),
                metadata={
                    **metadata,
                    "segment_index": 0,
                    "is_streaming": True,
                    "is_final": True,
                    "error": str(e)
                }
            )
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

            # Manage history based on configured strategy before generating
            history = self.manage_history(history, text)
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

                            # Check if streaming is enabled
                            enable_streaming = os.getenv("LLM_ENABLE_STREAMING", "true").lower() == "true"
                            
                            if enable_streaming and self.use_mlx:
                                # Use streaming for MLX
                                response = self.generate_response_mlx_streaming(user_text, session_id, metadata)
                                send_log(self.node, "INFO", f"Response streamed ({len(response)} chars)")
                            else:
                                # Use regular generation
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

                            # Check if streaming is enabled
                            enable_streaming = os.getenv("LLM_ENABLE_STREAMING", "true").lower() == "true"
                            
                            if enable_streaming and self.use_mlx:
                                # Use streaming for MLX
                                response = self.generate_response_mlx_streaming(user_text, session_id, metadata)
                                send_log(self.node, "INFO", f"Text-to-audio response streamed ({len(response)} chars)")
                            else:
                                # Use regular generation
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
