# dora-llama-cpp-python

A Dora node that provides access to LLaMA models using llama-cpp-python for efficient CPU/GPU inference.

## Features

- GPU acceleration support (CUDA on Linux, Metal on macOS)
- Easy integration with speech-to-text and text-to-speech pipelines  
- Configurable system prompts and activation words
- Lightweight CPU inference with GGUF models
- Thread-level CPU optimization
- Adjustable context window size

## Getting started

### Installation

```bash
uv venv -p 3.11 --seed
uv pip install -e .
```


## Usage

The node can be configured in your dataflow YAML file:

```yaml

# Using a local model

- id: dora-llama-cpp-python
  build: pip install -e path/to/dora-llama-cpp-python
  path: dora-llama-cpp-python
  inputs:
    text: source_node/text  # Input text to generate response for
  outputs:
    - text  # Generated response text
  env:
    MODEL_LOCAL_PATH: "./models/my-local-model.gguf"
    SYSTEM_PROMPT: "You're a very succinct AI assistant with short answers."
    ACTIVATION_WORDS: "what how who where you"
    MAX_TOKENS: "512"
    N_GPU_LAYERS: "35"     # Enable GPU acceleration
    N_THREADS: "4"         # CPU threads
    CONTEXT_SIZE: "4096"   # Maximum context window



# Using a HuggingFace model
- id: dora-llama-cpp-python
  build: pip install -e path/to/dora-llama-cpp-python
  path: dora-llama-cpp-python
  inputs:
    text: source_node/text  # Input text to generate response for
  outputs:
    - text  # Generated response text
  env:
    MODEL_NAME: "TheBloke/Llama-2-7B-Chat-GGUF"
    MODEL_FILE_PATTERN: "*Q4_K_M.gguf"
    SYSTEM_PROMPT: "You're a very succinct AI assistant with short answers."
    ACTIVATION_WORDS: "what how who where you"
    MAX_TOKENS: "512"
    N_GPU_LAYERS: "35"     # Enable GPU acceleration
    N_THREADS: "4"         # CPU threads
    CONTEXT_SIZE: "4096"   # Maximum context window
```

### Configuration Options

- `MODEL_PATH`: Path to your GGUF model file (default: "./models/llama-2-7b-chat.Q4_K_M.gguf")
- `SYSTEM_PROMPT`: Customize the AI assistant's personality/behavior
- `ACTIVATION_WORDS`: Space-separated list of words that trigger model response
- `MAX_TOKENS`: Maximum number of tokens to generate (default: 512)
- `N_GPU_LAYERS`: Number of layers to offload to GPU (default: 0, set to 35 for GPU acceleration)
- `N_THREADS`: Number of CPU threads to use (default: 4)
- `CONTEXT_SIZE`: Maximum context window size (default: 4096)

## Example: Speech Assistant Pipeline

This example shows how to create a conversational AI pipeline that:
1. Captures audio from microphone
2. Converts speech to text
3. Generates AI responses
4. Converts responses back to speech

```yaml
nodes:
  - id: dora-microphone
    build: pip install dora-microphone
    path: dora-microphone
    inputs:
      tick: dora/timer/millis/2000
    outputs:
      - audio

  - id: dora-vad
    build: pip install dora-vad
    path: dora-vad
    inputs:
      audio: dora-microphone/audio
    outputs:
      - audio
      - timestamp_start

  - id: dora-whisper
    build: pip install dora-distil-whisper
    path: dora-distil-whisper
    inputs:
      input: dora-vad/audio
    outputs:
      - text

  - id: dora-llama-cpp-python
    build: pip install -e .
    path: dora-llama-cpp-python
    inputs:
      text: dora-whisper/text
    outputs:
      - text
    env:
      MODEL_NAME: "TheBloke/Llama-2-7B-Chat-GGUF"
      MODEL_FILE_PATTERN: "*Q4_K_M.gguf"
      SYSTEM_PROMPT: "You're a helpful assistant."
      ACTIVATION_WORDS: "hey help what how"
      MAX_TOKENS: "512"
      N_GPU_LAYERS: "35"
      N_THREADS: "4"
      CONTEXT_SIZE: "4096"

  - id: dora-tts
    build: pip install dora-kokoro-tts
    path: dora-kokoro-tts
    inputs:
      text: dora-llama-cpp-python/text
    outputs:
      - audio
```

### Running the Example

```bash
dora build example.yml
dora run example.yml
```

## Contribution Guide

- Format with [ruff](https://docs.astral.sh/ruff/):

```bash
uv pip install ruff
uv run ruff check . --fix
```

- Lint with ruff:

```bash
uv run ruff check .
```

- Test with [pytest](https://github.com/pytest-dev/pytest)

```bash
uv pip install pytest
uv run pytest . # Test
```

## License

dora-llama-cpp-python is released under the MIT License