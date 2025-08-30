# Dora AEC (Acoustic Echo Cancellation) Node

A Dora node that provides acoustic echo cancellation and voice activity detection using macOS's native VoiceProcessingIO AudioUnit.

## Features

- **Acoustic Echo Cancellation (AEC)**: Removes echo from TTS playback for full-duplex conversation
- **Voice Activity Detection (VAD)**: Built-in hardware-accelerated VAD from macOS
- **Low Latency**: Direct access to macOS audio subsystem
- **Thread-Safe**: Proper synchronization for multi-threaded access

## Requirements

- macOS (uses native CoreAudio VoiceProcessingIO)
- Python 3.8+
- Native library: `libAudioCapture.dylib`

## Installation

```bash
pip install -e .
```

## Configuration

The node can be configured through the dataflow YAML:

```yaml
nodes:
  - id: aec_microphone
    path: dora-aec
    outputs:
      - audio
      - vad_status
      - log
    env:
      SAMPLE_RATE: 16000        # Audio sample rate
      CHANNELS: 1                # Number of channels (mono)
      BUFFER_SIZE: 512          # Audio buffer size in frames
      OUTPUT_FORMAT: "int16"     # Audio format
      AEC_ENABLED: true         # Enable echo cancellation
      VAD_ENABLED: true         # Enable voice activity detection
      AUTO_START: true          # Auto-start capture on initialization
      POLL_INTERVAL_MS: 10      # Polling interval in milliseconds
```

## Inputs

- `microphone`: Raw audio input from microphone (required)
  - 16kHz, mono, int16 format
  - This is the near-end signal containing speech + echo

- `reference_audio`: TTS output for echo cancellation (required for AEC)
  - The far-end signal that needs to be removed from microphone
  - Should be the same audio being played through speakers

- `control`: Control commands (start/stop/pause/resume/configure)
  - `start`: Start audio processing
  - `stop`: Stop audio processing
  - `pause`: Pause sending audio (processing continues)
  - `resume`: Resume sending audio
  - `configure`: Update configuration (requires dict with settings)

## Outputs

- `audio`: Echo-cancelled audio data (16kHz, mono, int16 format)
- `vad_status`: Voice activity detection status (boolean array)
- `log`: Logging messages

## Usage Example

### Basic Voice Chatbot with AEC

```yaml
nodes:
  # Raw microphone input (dynamic node)
  - id: microphone
    custom:
      source: ./microphone_input.py
      outputs:
        - audio

  # AEC processor with both near-end and far-end signals
  - id: aec_processor
    path: dora-aec
    inputs:
      microphone: microphone/audio        # Near-end signal (speech + echo)
      reference_audio: tts/audio          # Far-end signal (TTS output)
    outputs:
      - audio      # Echo-cancelled audio
      - vad_status
      - log
    env:
      AEC_ENABLED: true
      VAD_ENABLED: true

  - id: speech-monitor
    path: dora-speechmonitor
    inputs:
      microphone: aec_processor/audio     # Echo-cancelled audio
    outputs:
      - speech
      - status
      - log

  # No need for pause/resume control - AEC handles echo cancellation automatically
```

### Control via Python

```python
import pyarrow as pa
from dora import Node

node = Node()

# Send control commands
node.send_output("control", pa.array(["start"]))
node.send_output("control", pa.array(["pause"]))
node.send_output("control", pa.array(["resume"]))
node.send_output("control", pa.array(["stop"]))

# Configure at runtime
config = {
    "command": "configure",
    "args": {
        "AEC_ENABLED": False,
        "VAD_ENABLED": True
    }
}
node.send_output("control", pa.array([config]))
```

## Architecture

The AEC node processes two audio streams:

1. **Near-End Signal (Microphone Input)**
   - Raw audio from microphone containing user speech + echo
   - This is what needs to be cleaned

2. **Far-End Signal (Reference Audio)**
   - TTS output being played through speakers
   - Used as reference to identify and remove echo

### Components:

1. **Native Library (`libAudioCapture.dylib`)**
   - Swift implementation using CoreAudio VoiceProcessingIO
   - Hardware-accelerated adaptive echo cancellation
   - Outputs clean 16kHz mono audio with VAD status

2. **Python Wrapper (`aec_wrapper.py`)**
   - ctypes interface to native library
   - Thread-safe audio data retrieval
   - Memory management

3. **Dora Node (`main.py`)**
   - Combines microphone and reference signals
   - Event-driven Dora integration
   - Manages AEC processing pipeline

## Benefits Over Pause/Resume Approach

1. **Full-Duplex Conversation**: Can listen while TTS is playing
2. **No Latency**: No delay from pause/resume switching
3. **Natural Interruption**: User can interrupt at any time
4. **Hardware Acceleration**: Uses optimized macOS audio processing
5. **Built-in VAD**: Reduces computational load

## How AEC Works

Acoustic Echo Cancellation removes the echo of the far-end signal (TTS output) from the near-end signal (microphone input):

```
Microphone Input = User Speech + Echo of TTS
                         ↓
                   AEC Processing
                  (with TTS reference)
                         ↓
Output = User Speech Only (echo removed)
```

## Troubleshooting

### Library Not Found

If `libAudioCapture.dylib` is not found, the node will look in:
1. `./lib/libAudioCapture.dylib` (relative to module)
2. `~/home/VoiceDialogue/libraries/libAudioCapture.dylib`

Copy the library to one of these locations or modify the path in `main.py`.

### No Audio Output

Check that:
1. Both microphone and reference_audio inputs are connected
2. Microphone permissions are granted
3. The native library is loaded successfully
4. `AUTO_START` is set to `true` or send a `start` command

### Echo Still Present

Ensure:
1. `AEC_ENABLED` is set to `true`
2. Reference audio is properly connected to TTS output
3. Audio output and input devices are properly configured in macOS
4. The timing between reference and microphone signals is synchronized

## License

Apache-2.0