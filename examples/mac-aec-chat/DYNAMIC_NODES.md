# Dynamic Nodes in the Voice Assistant Pipeline

## Overview

This pipeline uses "dynamic" nodes which are resolved at runtime by Dora. These nodes are typically implemented as:
1. Native binaries provided by Dora
2. Python scripts in the current directory
3. Nodes registered with Dora's node registry

## Dynamic Nodes in voice-chat-with-aec.yml

### 1. MAC-AEC (mac-aec)
- **Path**: `mac_aec_simple_segmentation.py`
- **Implementation**: Python-based audio capture with VAD segmentation
- **Function**: Acoustic echo cancellation and VAD-based audio segmentation
- **Dependencies**: pyaudio, webrtcvad (install with: `pip install -r requirements.txt`)

### 2. Audio Player (audio-player)
- **Path**: `audio_player.py`
- **Implementation**: Python-based audio playback
- **Function**: Plays audio through system speakers
- **Dependencies**: Included in audio_player.py

### 3. Viewer (viewer)
- **Path**: `viewer.py`
- **Implementation**: Python-based pipeline monitor
- **Function**: Visual monitoring of pipeline events
- **Note**: Optional, can be removed if not needed

## Required Files

For the pipeline to work, you need:
- `mac_aec_simple_segmentation.py` - Audio capture with VAD (REQUIRED)
- `audio_player.py` - Audio playback implementation (REQUIRED)
- `viewer.py` - Optional monitoring interface (OPTIONAL)

## Installation

```bash
# Install audio dependencies
pip install -r requirements.txt

# Install Dora node dependencies
pip install -e ../../node-hub/dora-asr
pip install -e ../../node-hub/dora-qwen3
pip install -e ../../node-hub/dora-text-segmenter
pip install -e ../../node-hub/dora-primespeech
```

## Troubleshooting

If you get "node not found" errors:

1. **For mac-aec**: Ensure you're running on macOS with Dora properly installed
2. **For audio-player**: Ensure `audio_player.py` exists in the directory
3. **For viewer**: Either provide `viewer.py` or remove it from the pipeline

## Removing Optional Nodes

To run without the viewer, remove these lines from `voice-chat-with-aec.yml`:

```yaml
# Remove this entire block if you don't want visual monitoring
- id: viewer
  path: dynamic
  inputs:
    transcription: asr/transcription
    llm_output: qwen3-llm/text
    segment: text-segmenter/text_segment
    speech_started: mac-aec/speech_started
    speech_ended: mac-aec/speech_ended
```