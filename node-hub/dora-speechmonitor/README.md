# Speech Monitor Node

Real-time speech detection and segmentation node for voice conversation systems. Detects when users start/stop speaking and identifies complete questions through silence detection.

## Overview

The Speech Monitor acts as the voice activity detector (VAD) and speech segmenter in the conversation pipeline. It processes raw audio input, detects speech activity, and outputs segmented audio chunks along with control signals for conversation flow management.

## Features

- **Voice Activity Detection (VAD)**: Real-time detection using Silero VAD model
- **Multi-level Speech Events**: Detects speech start, end, and question completion
- **Configurable Thresholds**: Adjustable silence and amplitude thresholds
- **Pause/Resume Control**: Can be paused during AI responses to prevent feedback
- **Audio Segmentation**: Outputs clean audio segments for ASR processing
- **State Machine**: Robust state tracking for conversation flow
- **Task management**: Unique IDs for each speech segment
- **Configurable thresholds**: Fine-tune for your environment

## Outputs

- `speech_started`: Timestamp when speech begins
- `speech_ended`: Timestamp when speech stops
- `is_speaking`: Boolean stream of current speaking state
- `audio_segment`: Complete speech segment after speech ends
- `speech_probability`: Real-time VAD confidence (0.0-1.0)

## Configuration

Environment variables:

```bash
# Audio thresholds
MIN_AUDIO_AMPLITUDE=0.01        # Noise floor

# Time thresholds (milliseconds)
ACTIVE_FRAME_THRESHOLD_MS=100   # Speech start detection
USER_SILENCE_THRESHOLD_MS=1000  # User done speaking
SILENCE_THRESHOLD_MS=300        # Speech segment end
AUDIO_FRAMES_THRESHOLD_MS=10000 # Max segment length

# VAD configuration
VAD_THRESHOLD=0.7               # Confidence threshold
VAD_ENABLED=true                # Enable/disable VAD

# Audio settings
SAMPLE_RATE=16000               # Sample rate (Hz)
```

## Usage

```yaml
nodes:
  - id: speech-monitor
    build: pip install -e ../../node-hub/dora-speechmonitor
    path: dora-speechmonitor
    inputs:
      audio: microphone/audio
    outputs:
      - speech_started
      - speech_ended
      - is_speaking
      - audio_segment
      - speech_probability
```

## Key Improvements over dora-vad

- 10x lower latency (100ms vs 1000ms)
- Proper state tracking
- Pre-speech buffer captures speech onset
- Real-time confidence output
- Task-based architecture for concurrent sessions