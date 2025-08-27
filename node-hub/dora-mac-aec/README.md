# Dora macOS AEC Node

Hardware-accelerated Acoustic Echo Cancellation (AEC) node for Dora using macOS VoiceProcessingIO Audio Unit.

## Features

- 🎤 **Hardware Echo Cancellation** - Uses macOS native VoiceProcessingIO for zero-latency AEC
- 🔊 **Voice Activity Detection (VAD)** - Built-in voice detection with each audio chunk
- 📊 **Comprehensive Metrics** - Real-time audio quality metrics and statistics
- 🐛 **Advanced Debugging** - Detailed logging, audio saving, and performance monitoring
- 🔧 **Dynamic Configuration** - Runtime configurable via environment variables or control commands
- 🚀 **High Performance** - Direct native library integration with minimal overhead
- 🎯 **Speech Monitor Compatible** - Outputs float32 audio arrays ready for speech-monitor node

## Installation

```bash
# Install from the node-hub directory
cd node-hub/dora-mac-aec
pip install -e .

# Or install with development dependencies
pip install -e ".[dev]"
```

## Quick Start

### Basic Usage in Dataflow

```yaml
nodes:
  - id: mac-aec
    build: pip install -e ../../node-hub/dora-mac-aec
    path: dora-mac-aec
    outputs:
      - audio      # Float32 audio samples
      - vad        # Voice activity detection
      - status     # Node status updates
      - metrics    # Audio metrics (optional)
    env:
      SAMPLE_RATE: 16000
      AUTO_START: true
      LOG_LEVEL: INFO
```

### With Speech Monitor

```yaml
nodes:
  - id: mac-aec
    path: dora-mac-aec
    outputs:
      - audio
      - vad
      - status
      - metrics
    env:
      AUTO_START: true
      ENABLE_METRICS: true
      LOG_INTERVAL: 5.0
  
  - id: speech-monitor
    path: dora-speechmonitor
    inputs:
      audio: mac-aec/audio
      vad: mac-aec/vad  # Optional VAD input
    outputs:
      - speech_segment
```

## Configuration

### Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| **Audio Settings** | | |
| `SAMPLE_RATE` | `16000` | Audio sample rate (Hz) |
| `CHUNK_SIZE` | `512` | Audio chunk size (samples) |
| **Features** | | |
| `ENABLE_AEC` | `true` | Enable echo cancellation |
| `ENABLE_VAD` | `true` | Enable voice activity detection |
| `AUTO_START` | `true` | Auto-start capture on node init |
| **Logging** | | |
| `LOG_LEVEL` | `INFO` | Log level (DEBUG/INFO/WARNING/ERROR) |
| `LOG_INTERVAL` | `5.0` | Status update interval (seconds) |
| `ENABLE_METRICS` | `true` | Enable metrics collection |
| **Debug** | | |
| `DEBUG_AUDIO` | `false` | Enable detailed audio debugging |
| `SAVE_AUDIO` | `false` | Save audio chunks to disk |
| `AUDIO_DIR` | `./debug_audio` | Directory for saved audio |
| **Library** | | |
| `AEC_LIB_PATH` | `(auto)` | Path to libAudioCapture.dylib |

### Control Commands

Send control commands via the `control` input:

```python
# Start capture
{"command": "start"}

# Stop capture
{"command": "stop"}

# Get current status
{"command": "get_status"}

# Reset metrics
{"command": "reset_metrics"}

# Update configuration
{
    "command": "configure",
    "args": {
        "log_level": "DEBUG",
        "enable_metrics": true
    }
}
```

## Outputs

### 1. Audio Output
- **Type**: `pa.array(float32)`
- **Format**: Normalized float32 samples in range [-1.0, 1.0]
- **Rate**: Continuous stream when audio available (~50Hz)

### 2. VAD Output
- **Type**: `pa.array([bool])`
- **Values**: `True` = voice detected, `False` = silence/noise

### 3. Status Output
- **Type**: `pa.array([json_string])`
- **Format**: JSON with status information
```json
{
    "status": "started|stopped|current",
    "timestamp": 1234567890.123,
    "frame_count": 1000,
    "details": {...}
}
```

### 4. Metrics Output (Optional)
- **Type**: `pa.array([json_string])`
- **Format**: JSON with audio metrics
```json
{
    "current_rms": 0.0234,
    "current_peak": 0.156,
    "rms_mean": 0.018,
    "vad_positive_rate": 0.45,
    "audio_quality": "good",
    "total_chunks": 5000,
    ...
}
```

## Debugging Features

### 1. Enhanced Logging

```bash
# Enable debug logging
LOG_LEVEL=DEBUG dora start dataflow.yml

# Log to file
LOG_FILE=/tmp/mac-aec.log dora start dataflow.yml
```

### 2. Audio Metrics

When `ENABLE_METRICS=true`, the node tracks:
- RMS energy levels
- Peak amplitudes
- VAD activation rate
- Clipping detection
- Silence detection
- Audio quality assessment

### 3. Audio Saving

Save audio chunks for offline analysis:

```bash
SAVE_AUDIO=true AUDIO_DIR=./debug_audio dora start dataflow.yml
```

Files are saved as: `chunk_000001_vad1.npy` (numpy format)

### 4. Real-time Monitoring

Monitor the node in real-time:

```python
# In another Python script
import json

# Subscribe to metrics output
for event in node.subscribe("mac-aec/metrics"):
    metrics = json.loads(event["value"][0])
    print(f"RMS: {metrics['current_rms']:.4f}, VAD: {metrics['vad_positive_rate']:.1%}")
```

## How It Works

### Architecture

```
macOS Audio Input
       ↓
VoiceProcessingIO Audio Unit
       ↓
  [AEC + AGC + NS]  ← Hardware accelerated
       ↓
   Native Library (libAudioCapture.dylib)
       ↓
   Python Wrapper (ctypes)
       ↓
   Dora Node
       ↓
  [audio, vad, status, metrics]
```

### Key Components

1. **Native Library** (`libAudioCapture.dylib`)
   - Swift implementation using CoreAudio
   - Configures VoiceProcessingIO Audio Unit
   - Provides C interface for Python

2. **AEC Wrapper** (`aec_wrapper.py`)
   - Python ctypes interface to native library
   - Thread-safe audio capture
   - Memory management

3. **Main Node** (`main.py`)
   - Dora node implementation
   - Configuration management
   - Metrics collection
   - Debug features

4. **Utilities**
   - `logger.py` - Enhanced logging
   - `metrics.py` - Audio metrics analysis

## Comparison with Other Audio Capture Methods

| Feature | mac-aec | PyAudio | dora-microphone |
|---------|---------|---------|-----------------|
| Echo Cancellation | ✅ Hardware | ❌ None | ❌ None |
| Voice Activity Detection | ✅ Built-in | ❌ Needs VAD | ❌ Needs VAD |
| Latency | ✅ Minimal | ⚠️ Variable | ⚠️ Variable |
| CPU Usage | ✅ Low | ⚠️ Medium | ⚠️ Medium |
| macOS Native | ✅ Yes | ❌ No | ❌ No |
| Cross-platform | ❌ macOS only | ✅ Yes | ✅ Yes |

## Examples

### Example 1: Simple Voice Assistant

```yaml
nodes:
  - id: mac-aec
    path: dora-mac-aec
    outputs: [audio, vad]
    
  - id: asr
    path: dora-whisper
    inputs:
      audio: mac-aec/audio
    outputs: [text]
    
  - id: llm
    path: dora-llm
    inputs:
      text: asr/text
    outputs: [response]
    
  - id: tts
    path: dora-tts
    inputs:
      text: llm/response
    outputs: [audio]
```

### Example 2: Audio Quality Monitor

```yaml
nodes:
  - id: mac-aec
    path: dora-mac-aec
    outputs: [audio, vad, metrics]
    env:
      ENABLE_METRICS: true
      LOG_INTERVAL: 1.0
      DEBUG_AUDIO: true
      
  - id: quality-monitor
    path: ./audio_quality_monitor.py
    inputs:
      metrics: mac-aec/metrics
    outputs: [alert]
```

### Example 3: Debug Recording

```yaml
nodes:
  - id: mac-aec
    path: dora-mac-aec
    outputs: [audio, vad, status]
    env:
      SAVE_AUDIO: true
      AUDIO_DIR: ./recordings
      DEBUG_AUDIO: true
      LOG_LEVEL: DEBUG
```

## Troubleshooting

### Issue: Library not found
```
Error: Native library not found at .../libAudioCapture.dylib
```
**Solution**: The node will automatically copy the library from dora-aec if available. Ensure dora-aec is installed.

### Issue: No audio captured
```
Warning: No audio data available
```
**Solution**: 
1. Check microphone permissions in System Preferences
2. Verify no other app is using the microphone exclusively
3. Try `LOG_LEVEL=DEBUG` for more details

### Issue: High CPU usage
**Solution**: 
1. Disable metrics: `ENABLE_METRICS=false`
2. Increase log interval: `LOG_INTERVAL=30.0`
3. Disable debug features: `DEBUG_AUDIO=false`

### Issue: VAD not working
```
VAD always returns False
```
**Solution**: VAD requires clear speech. Check:
1. Microphone gain settings
2. Background noise levels
3. Try speaking louder/clearer

## Performance

Typical performance on Apple Silicon Mac:
- **Latency**: < 20ms (hardware AEC)
- **CPU Usage**: < 2% (M1 Pro)
- **Memory**: ~50MB
- **Audio Quality**: 16-bit, 16kHz (configurable)

## Requirements

- macOS 11.0 or later
- Python 3.8+
- Apple Silicon or Intel Mac
- Microphone permissions granted

## License

MIT License - See LICENSE file for details

## Contributing

Contributions welcome! Please ensure:
1. Code follows existing style
2. Tests pass (when added)
3. Documentation is updated
4. Changes are backwards compatible

## Credits

Built on top of the VoiceDialogue AEC implementation and Dora framework.