# Audio Player Node (Dynamic)

Real-time audio playback node with buffer management and status reporting. Handles TTS audio output with buffer monitoring for backpressure control.

## Overview

The Audio Player is a dynamic Dora node that plays audio from TTS systems while providing real-time buffer status feedback. This enables backpressure control to prevent buffer overflow and ensure smooth, continuous playback.

## Features

- **Real-time Audio Playback**: Low-latency audio output
- **Buffer Management**: Circular buffer with configurable size
- **Status Reporting**: Real-time buffer fullness percentage
- **Non-blocking Operation**: Asynchronous audio playback
- **Clear Display Mode**: Optional screen clearing for clean output
- **Pause/Resume Control**: External control for playback management

## Dynamic Node Implementation

As a dynamic node, the audio player is implemented directly in the dataflow YAML:

```python
# Python implementation runs as dynamic node
import sounddevice as sd
import numpy as np
from collections import deque
import threading
```

## Inputs

| Input | Type | Description |
|-------|------|-------------|
| `audio` | Audio array | PCM audio data from TTS (16kHz mono) |
| `control` | String | Control commands: "pause", "resume", "clear", "status" |

## Outputs

| Output | Type | Description |
|--------|------|-------------|
| `buffer_status` | Dict | Buffer fullness percentage and statistics |
| `status` | Dict | Playback state and metrics |

## Configuration

```yaml
nodes:
  - id: audio-player
    path: dynamic
    inputs:
      audio: primespeech/audio
      control: external/control
    outputs:
      - buffer_status
      - status
    env:
      NODE_TIMEOUT_MS: 1000  # Event loop timeout (controls reporting frequency)
      BUFFER_SIZE_SECONDS: 5  # Audio buffer size in seconds
      SAMPLE_RATE: 16000      # Audio sample rate
      CLEAR_SCREEN: true      # Clear screen for clean display
```

## Buffer Status Format

```json
{
    "buffer_percentage": 65.5,
    "buffer_duration": 3.2,
    "buffer_samples": 51200,
    "max_buffer_samples": 80000,
    "is_playing": true,
    "timestamp": 1234567890.123
}
```

## Implementation Details

### Buffer Management
```python
# Circular buffer using deque
audio_buffer = deque(maxlen=BUFFER_SIZE_SECONDS * SAMPLE_RATE)

# Add audio
audio_buffer.extend(audio_chunk)

# Calculate fullness
buffer_percentage = (len(audio_buffer) / audio_buffer.maxlen) * 100
```

### Playback Thread
```python
def playback_thread():
    while running:
        if len(audio_buffer) > min_chunk_size and not paused:
            chunk = extract_chunk(audio_buffer)
            stream.write(chunk)
        else:
            time.sleep(0.01)  # Small delay when buffer empty
```

### Status Reporting
```python
# Report buffer status at regular intervals
if time.time() - last_report > report_interval:
    send_buffer_status()
    last_report = time.time()
```

## Display Modes

### Standard Mode
```
[INFO] Buffer: 65.5% (3.2s)
[INFO] Playing audio chunk: 8000 samples
[INFO] Buffer: 58.2% (2.8s)
```

### Clear Screen Mode (CLEAR_SCREEN=true)
```
════════════════════════════════════════
       Audio Player Status
════════════════════════════════════════
Buffer: ████████░░ 75.3% (3.8s)
State:  ▶ Playing
Chunks: 12 processed
```

## Control Commands

- **`pause`**: Pause audio playback (buffer continues filling)
- **`resume`**: Resume audio playback
- **`clear`**: Clear audio buffer
- **`status`**: Force status report
- **`reset`**: Clear buffer and reset state

## Integration with Backpressure

```
primespeech → audio-player → conversation-controller
                   ↓                    ↑
            buffer_status ──────────────┘
```

1. Audio player receives TTS audio
2. Monitors buffer fullness
3. Reports status to conversation-controller
4. Controller pauses text-segmenter if buffer > 70%
5. Resumes when buffer < 50%

## Best Practices

1. **Buffer Size**:
   - 3-5 seconds for normal conversation
   - 5-10 seconds for slow networks
   - 1-2 seconds for real-time response

2. **Report Frequency**:
   - Every 100-500ms for responsive control
   - Every 1000ms for lower overhead

3. **Minimum Playback**:
   - Wait for 0.1-0.5s of audio before starting
   - Prevents choppy playback

4. **Thread Safety**:
   - Use locks when accessing buffer
   - Separate playback and receive threads

## Performance Considerations

- **CPU Usage**: ~1-3% for audio playback
- **Memory**: Buffer size × sample rate × 2 bytes
- **Latency**: < 50ms from receive to playback
- **Report Overhead**: < 0.1% CPU per report

## Troubleshooting

| Issue | Solution |
|-------|----------|
| No audio output | Check audio device, verify sample rate |
| Choppy playback | Increase buffer size or min chunk |
| High latency | Decrease buffer size |
| Buffer overflow | Enable backpressure control |
| CPU spikes | Increase chunk size, reduce report frequency |

## Audio Format Requirements

- **Sample Rate**: 16000 Hz (configurable)
- **Channels**: Mono
- **Format**: Float32 or Int16
- **Chunk Size**: Flexible (typically 1600-8000 samples)

## Example Integration

```yaml
# Complete audio pipeline
nodes:
  - id: primespeech
    outputs:
      - audio  # 16kHz mono float32
      
  - id: audio-player
    path: dynamic
    inputs:
      audio: primespeech/audio
    outputs:
      - buffer_status
    env:
      BUFFER_SIZE_SECONDS: 5
      NODE_TIMEOUT_MS: 200  # Report 5 times per second
      
  - id: conversation-controller
    inputs:
      buffer_status: audio-player/buffer_status
    outputs:
      - segment_control  # Pause/resume for text-segmenter
```

## Monitoring Metrics

Track these metrics for optimal performance:

- **Average Buffer %**: Should stay 40-60%
- **Peak Buffer %**: Should rarely hit 100%
- **Underrun Count**: Buffer empty events
- **Playback Continuity**: Percentage of continuous playback

## Dependencies

- `sounddevice`: Audio playback
- `numpy`: Audio data processing
- `threading`: Asynchronous playback
- `collections.deque`: Circular buffer
- `pyarrow`: Dora communication