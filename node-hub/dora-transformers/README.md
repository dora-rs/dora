# dora-transformers

A Dora node that provides access to Hugging Face transformer models for efficient text generation and chat completion.

## Features

- Multi-platform GPU acceleration support (CUDA, CPU, MPS)
- Memory-efficient model loading with 8-bit quantization
- Seamless integration with speech-to-text and text-to-speech pipelines
- Configurable system prompts and activation words
- Support for various transformer models
- Conversation history management
- Optimized for both small and large language models

## Getting Started

### Installation

```bash
uv venv -p 3.11 --seed
uv pip install -e .
```

## Usage

Configure the node in your dataflow YAML file:

```yaml
- id: dora-transformers
  build: pip install -e path/to/dora-transformers
  path: dora-transformers
  inputs:
    text: source_node/text  # Input text to generate response for
  outputs:
    - text  # Generated response text
  env:
    MODEL_NAME: "Qwen/Qwen2.5-0.5B-Instruct"  # Model from Hugging Face
    SYSTEM_PROMPT: "You're a very succinct AI assistant with short answers."
    ACTIVATION_WORDS: "what how who where you"
    MAX_TOKENS: "128"  # Reduced for concise responses
    DEVICE: "cuda"  # Use "cpu" for CPU, "cuda" for NVIDIA GPU, "mps" for Apple Silicon
    ENABLE_MEMORY_EFFICIENT: "true"  # Enable 8-bit quantization and memory optimizations
    TORCH_DTYPE: "float16"  # Use half precision for better memory efficiency
```

### Configuration Options

- `MODEL_NAME`: Hugging Face model identifier (default: "Qwen/Qwen2.5-0.5B-Instruct")
- `SYSTEM_PROMPT`: Customize the AI assistant's personality/behavior
- `ACTIVATION_WORDS`: Space-separated list of words that trigger model response
- `MAX_TOKENS`: Maximum number of tokens to generate (default: 128)
- `DEVICE`: Computation device to use (default: "cpu")
- `ENABLE_MEMORY_EFFICIENT`: Enable memory optimizations (default: "true")
- `TORCH_DTYPE`: Model precision type (default: "auto")

### Memory Management

The node includes several memory optimization features:
- 8-bit quantization for CUDA devices
- Automatic CUDA cache clearing
- Memory-efficient model loading
- Half-precision support

## Example: Voice Assistant Pipeline

Create a conversational AI pipeline that:
1. Captures audio from microphone
2. Converts speech to text
3. Generates AI responses using transformers
4. Converts responses back to speech

```yaml
nodes:
  - id: dora-microphone
    build: pip install -e ../../node-hub/dora-microphone
    path: dora-microphone
    inputs:
      tick: dora/timer/millis/2000
    outputs:
      - audio

  - id: dora-vad
    build: pip install -e ../../node-hub/dora-vad
    path: dora-vad
    inputs:
      audio: dora-microphone/audio
    outputs:
      - audio
      - timestamp_start

  - id: dora-distil-whisper
    build: pip install -e ../../node-hub/dora-distil-whisper
    path: dora-distil-whisper
    inputs:
      input: dora-vad/audio
    outputs:
      - text
    env:
      TARGET_LANGUAGE: english

  - id: dora-transformers
    build: pip install -e ../../node-hub/dora-transformers
    path: dora-transformers
    inputs:
      text: dora-distil-whisper/text
    outputs:
      - text
    env:
      MODEL_NAME: "Qwen/Qwen2.5-0.5B-Instruct"
      SYSTEM_PROMPT: "You are an AI assistant that gives extremely concise responses, never more than one or two sentences. Always be direct and to the point."
      ACTIVATION_WORDS: "what how who where you"
      MAX_TOKENS: "128"
      DEVICE: "cuda"
      ENABLE_MEMORY_EFFICIENT: "true"
      TORCH_DTYPE: "float16"

  - id: dora-kokoro-tts
    build: pip install -e ../../node-hub/dora-kokoro-tts
    path: dora-kokoro-tts
    inputs:
      text: dora-transformers/text
    outputs:
      - audio
```

### Running the Example

```bash
dora build test.yml
dora run test.yml
```

### Troubleshooting

If you encounter CUDA out of memory errors:
1. Set `DEVICE: "cpu"` for CPU-only inference
2. Enable memory optimizations with `ENABLE_MEMORY_EFFICIENT: "true"`
3. Use a smaller model or reduce `MAX_TOKENS`
4. Set `TORCH_DTYPE: "float16"` for reduced memory usage

## Contribution Guide

Format with [ruff](https://docs.astral.sh/ruff/):
```bash
uv pip install ruff
uv run ruff check . --fix
```

Lint with ruff:
```bash
uv run ruff check .
```

Test with [pytest](https://github.com/pytest-dev/pytest):
```bash
uv pip install pytest
uv run pytest . # Test
```

## License

dora-transformers is released under the MIT License
