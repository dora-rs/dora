# dora-llama-cpp-python

A Dora node that provides access to LLaMA-based models using either llama.cpp or Hugging Face backends for text generation.

## Features

- Supports both llama.cpp (CPU) and Hugging Face (CPU/GPU) backends
- Easy integration with speech-to-text and text-to-speech pipelines  
- Configurable system prompts and activation words
- Chat history support with Hugging Face models
- Lightweight CPU inference with GGUF models

## Getting started

### Installation

```bash
uv venv -p 3.11 --seed
uv pip install -e .
```

## Usage

The node can be configured in your dataflow YAML file:

```yaml
- id: dora-llama-cpp-python
  build: pip install -e path/to/dora-llama-cpp-python
  path: dora-llama-cpp-python
  inputs:
    text: source_node/text  # Input text to generate response for
  outputs:
    - text  # Generated response text
  env:
    MODEL_BACKEND: "llama-cpp"  # or "huggingface"
    SYSTEM_PROMPT: "You're a very succinct AI assistant with short answers."
    ACTIVATION_WORDS: "what how who where you" # Space-separated activation words
```

### Configuration Options

- `MODEL_BACKEND`: Choose between:
  - `llama-cpp`: Uses GGUF models via llama.cpp (CPU-optimized, default)
  - `huggingface`: Uses Hugging Face Transformers models

- `SYSTEM_PROMPT`: Customize the AI assistant's personality/behavior
- `ACTIVATION_WORDS`: Space-separated list of words that trigger model response

## Example yml

### Basic Speech-to-Text-to-Speech Pipeline

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
      MODEL_BACKEND: llama-cpp
      SYSTEM_PROMPT: "You're a helpful assistant."
      ACTIVATION_WORDS: "hey help what how"

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

dora-llama-cpp-python's code is released under the MIT License
