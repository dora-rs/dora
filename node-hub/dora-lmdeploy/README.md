# dora-lmdeploy

## Getting started

- Install it with uv:

```bash
uv venv -p 3.11 --seed
uv pip install -e .
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

## YAML Specification

This node can be used as follows:

```yaml
- id: dora-lmdeploy
  build: pip install dora-lmdeploy
  path: dora-lmdeploy
  inputs:
    text:
      source: dora-distil-whisper/text  # Optional text input
      queue_size: 1
    image:
      source: camera/image  # Optional image input
      queue_size: 1
  outputs:
    - text  # Model's response
  env:
    MODEL_NAME: "internlm/internlm2-7b"  # Default model, can be changed
    MAX_LENGTH: 2048  # Maximum length of generated text
    TEMPERATURE: 0.7  # Sampling temperature
    TOP_P: 0.9  # Top-p sampling parameter
    SYSTEM_PROMPT: "You are a helpful AI assistant."  # Optional system prompt
    DEFAULT_QUESTION: "Describe this image"  # Default question when no text input is provided
    TURBOMIND_CACHE_DIR: "./workspace"  # Cache directory for Turbomind
    TURBOMIND_TP: 1  # Tensor parallelism degree
    TURBOMIND_GPU_MEMORY_FRACTION: 0.8  # GPU memory fraction to use
```

### Available Models
The node supports various models that can be specified in the `MODEL_NAME` environment variable. Some examples:
- `internlm/internlm2-7b`
- `internlm/internlm2-20b`
- `internlm/internlm2-7b-chat`
- `internlm/internlm2-20b-chat`
- `Qwen/Qwen2-7B`
- `Qwen/Qwen2-14B`

### Input/Output
- **Inputs**:
  - `text`: Optional text input for text-only or multimodal tasks. If not provided, uses DEFAULT_QUESTION
  - `image`: Optional image input for vision-language tasks. Supports multiple formats:
    - Raw image formats: bgr8, rgb8
    - File formats: jpeg, jpg, jpe, bmp, webp, png
- **Outputs**:
  - `text`: The model's generated response with metadata containing the image_id

### Environment Variables
- `MODEL_NAME`: Name of the model to use (default: "internlm/internlm2-7b")
- `MAX_LENGTH`: Maximum length of generated text (default: 2048)
- `TEMPERATURE`: Sampling temperature (default: 0.7)
- `TOP_P`: Top-p sampling parameter (default: 0.9)
- `SYSTEM_PROMPT`: Optional system prompt to guide model behavior
- `DEFAULT_QUESTION`: Default question to use when no text input is provided
- `TURBOMIND_CACHE_DIR`: Directory for storing Turbomind cache (default: "./workspace")
- `TURBOMIND_TP`: Tensor parallelism degree (default: 1)
- `TURBOMIND_GPU_MEMORY_FRACTION`: GPU memory fraction to use (default: 0.8)

### Features
- Efficient inference using LMDeploy's Turbomind engine
- Support for multimodal inputs (text + image)
- Conversation history tracking
- Automatic image format conversion and processing
- Configurable model parameters and generation settings
- GPU memory optimization through Turbomind

## Examples

## License

dora-lmdeploy's code are released under the MIT License
