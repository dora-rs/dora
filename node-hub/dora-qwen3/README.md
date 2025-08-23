# Qwen3 LLM Node

High-performance Large Language Model node featuring Qwen3 with automatic hardware optimization. Supports both MLX (Apple Silicon) and GGUF (cross-platform) backends.

## Overview

The Qwen3 node provides intelligent conversational AI capabilities optimized for real-time voice interactions. It automatically detects hardware and uses the best available backend - MLX for Apple Silicon or GGUF for other systems.

## Features

- **Automatic Hardware Detection**: Selects MLX on Apple Silicon, GGUF elsewhere
- **Multiple Model Sizes**: 1B to 70B parameters
- **Streaming Generation**: Token-by-token output for responsive TTS
- **Conversation Context**: Maintains chat history
- **Bilingual Support**: Optimized for Chinese and English
- **System Prompts**: Customizable assistant personality
- **Temperature Control**: Adjustable response creativity

## Installation

```bash
pip install -e .
```

Or directly from the directory:
```bash
cd dora-qwen3
pip install .
```

## Usage

### In a Dora Dataflow

```yaml
nodes:
  - id: qwen3-llm
    operator: dora-qwen3
    inputs:
      text: chat-client/text
      control: chat-client/control
    outputs:
      - response
      - status
    env:
      USE_MLX: auto  # auto, true, or false
      ENABLE_THINKING: false
      MAX_TOKENS: 2048
```

### Environment Variables

- `USE_MLX`: Control MLX usage (auto/true/false)
- `MLX_MODEL`: MLX model to use (default: Qwen/Qwen3-8B-MLX-4bit)
- `GGUF_MODEL`: GGUF model repository (default: Qwen/Qwen3-8B-GGUF)
- `GGUF_MODEL_FILE`: GGUF model file (default: Qwen3-8B-Q4_K_M.gguf)
- `ENABLE_THINKING`: Enable thinking mode with <think> tags
- `MAX_TOKENS`: Maximum tokens to generate
- `TEMPERATURE`: Generation temperature
- `N_GPU_LAYERS`: GPU layers for GGUF (0 for CPU only)
- `N_THREADS`: Number of CPU threads for GGUF
- `CONTEXT_SIZE`: Context window size

## Models

### MLX (Apple Silicon)
- Model: `Qwen/Qwen3-8B-MLX-4bit`
- Performance: ~0.3s average response time
- Memory: ~4.5GB

### GGUF (CPU/GPU)
- Model: `Qwen/Qwen3-8B-GGUF`
- Performance: ~1.3s average response time
- Memory: ~5GB

## License

MIT