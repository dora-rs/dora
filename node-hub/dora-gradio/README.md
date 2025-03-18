# Dora Gradio UI Interface

A versatile UI interface for Dora-rs that provides both text and voice input capabilities using Gradio.

## Features

- **Text Input**: Direct text input through a chat-like interface
- **Voice Input**: Real-time audio transcription using Whisper
- **Multiple Output Channels**: 
  - `text_input`: For direct text messages
  - `transcribed_text`: For transcribed audio content
- **Clean Interface**: Simple and intuitive UI with clear sections
- **Auto Port Management**: Automatically handles port conflicts

## Installation

1. Using `uv` (recommended):
```bash
uv venv -p 3.11 --seed
uv pip install -e .
```

2. Using `pip`:
```bash
python -m venv .venv
source .venv/bin/activate
pip install -e .
```

## Usage

### 1. Web Interface

The interface will be available at: `http://localhost:7860`

### 2. As a Dora Node

1. Create a YAML configuration:
```yaml
nodes:
  - id: ui
    build: pip install -e .
    path: dora-gradio
    outputs:
      - text_input     # Text from chat interface
      - transcribed_text    # Transcribed text from audio
```

2. Run with Dora:
```bash
dora run your_config.yml --uv
```

### 3. Integration Examples

#### Text Processing Pipeline
```yaml
nodes:
  - id: ui
    build: pip install -e .
    path: dora-gradio
    outputs:
      - text_input
      - transcribed_text

  - id: text_processor
    build: pip install -e path/to/processor
    path: text-processor
    inputs:
      text: ui/text_input
```

#### Voice Assistant Pipeline
```yaml
nodes:
  - id: ui
    build: pip install -e .
    path: dora-gradio
    outputs:
      - transcribed_text  # Use voice input

  - id: llm
    build: pip install -e path/to/llm
    path: llm-node
    inputs:
      prompt: ui/transcribed_text
```

## Interface Features

### Text Input Section
- Chat-like interface for text input
- History tracking
- Immediate output through `text_input` channel

### Voice Input Section
- Real-time audio streaming
- Automatic transcription using Whisper
- Noise filtering and audio preprocessing
- Output through `transcribed_text` channel
- Clear button to reset recording
- Status indicators for recording/transcription state

### Controls
- Send button for voice transcriptions
- Clear button for resetting inputs
- Stop Server button for graceful shutdown

## Development

### Code Formatting
```bash
uv pip install ruff
uv run ruff check . --fix
```

### Linting
```bash
uv run ruff check .
```

### Testing
```bash
uv pip install pytest
uv run pytest .
```

## System Requirements

- Python ≥ 3.11
- CUDA-compatible GPU (optional, for faster transcription)
- Required ports:
  - 7860 (Gradio interface)

## Known Limitations

- Only supports English language for voice transcription
- Requires manual port management if multiple instances needed
- Voice transcription may be slower on CPU-only systems

## License

dora-gradio's code are released under the MIT License

## Contributing

1. Fork the repository
2. Create your feature branch
3. Commit your changes
4. Push to the branch
5. Create a new Pull Request
