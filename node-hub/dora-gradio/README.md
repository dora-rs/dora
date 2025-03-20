# Dora Gradio UI Interface

A versatile UI interface for Dora-rs that provides text, audio, and video input capabilities using Gradio.

## Features

- **Text Input**: Direct text input through a chat-like interface
- **Audio Input**: Real-time audio streaming in 16kHz format
- **Video Input**: WebRTC camera streaming at 640x480
- **Multiple Output Channels**: 
  - `text`: For direct text messages
  - `audio`: For raw audio stream
  - `image`: For camera feed
- **Clean Interface**: Simple and intuitive UI with tabbed sections
- **Auto Port Management**: Automatically handles port conflicts

## Installation

Using `pip`:
```bash
python -m venv .venv
source .venv/bin/activate
pip install -e .
```

## Usage

### 1. Web Interface

The interface will be available at: `http://localhost:7860`

### 2. As a Dora Node

Create a YAML configuration:
```yaml
nodes:
  - id: ui
    build: pip install -e .
    path: dora-gradio
    outputs:
      - text     # Text from chat interface
      - audio    # Raw audio stream
      - image    # Camera feed
    env:
      VIRTUAL_ENV: path to your venv   # comment this if not using venv

```

Run with Dora:
```bash
dora run demo.yml
```

### 3. Integration Examples

#### Video Processing Pipeline
```yaml
nodes:
  - id: ui
    build: pip install -e .
    path: dora-gradio
    outputs:
      - image    # Camera feed

  - id: video_processor
    build: pip install -e path/to/processor
    path: video-processor
    inputs:
      video: ui/image
```

#### Audio Processing Pipeline
```yaml
nodes:
  - id: ui
    build: pip install -e .
    path: dora-gradio
    outputs:
      - audio    # Raw audio stream

  - id: audio_processor
    build: pip install -e path/to/processor
    path: audio-processor
    inputs:
      audio: ui/audio
```

### 4. Demo Example

Here's a complete demo pipeline using the UI with visualization and audio processing:

```yaml
nodes:
  - id: ui
    build: pip install -e .
    path: dora-gradio
    outputs:
      - text     # Text messages
      - audio    # Raw audio stream
      - image    # Camera feed

  - id: plot
    build: pip install dora-rerun
    path: dora-rerun
    inputs:
      text_input: ui/text
      audio: ui/audio
      image: ui/image

  - id: dora-vad
    build: pip install -e path/to/dora-vad
    path: dora-vad
    inputs:
      audio: ui/audio

  - id: dora-distil-whisper
    build: pip install -e path/to/dora-distil-whisper
    path: dora-distil-whisper
    inputs:
      audio: ui/audio
```

This demo showcases:
- Real-time visualization with dora-rerun
- Voice Activity Detection with dora-vad
- Speech processing with dora-distil-whisper

## Interface Features

### Camera Tab
- Real-time WebRTC video streaming
- Fixed 640x480 resolution
- BGR8 color format
- Automatic timestamp synchronization

### Audio and Text Input Tab
- Chat-like interface for text input
- Real-time audio streaming (16kHz, mono)
- Status indicators for streaming state
- Immediate output through respective channels

### Controls
- Send Text button for chat messages
- Stop Server button for graceful shutdown

## System Requirements

- Python ≥ 3.10
- Required ports:
  - 7860 (Gradio interface)

## Known Limitations

- Fixed video resolution (640x480)
- Fixed audio sample rate (16kHz)
- Requires port 7860 to be available

## License

dora-gradio's code are released under the MIT License
