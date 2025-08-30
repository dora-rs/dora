# Voice Chatbot with Acoustic Echo Cancellation (AEC)

An advanced voice chatbot implementation using Dora's event-driven architecture with hardware-accelerated acoustic echo cancellation for full-duplex conversation.

## Overview

This implementation enhances the original voice chatbot by integrating acoustic echo cancellation (AEC) using macOS's native VoiceProcessingIO AudioUnit. This eliminates the need for manual pause/resume control during TTS playback, enabling natural full-duplex conversation where users can interrupt at any time.

## Key Improvements Over Pause/Resume Approach

1. **Full-Duplex Conversation**: Listen and speak simultaneously
2. **Zero Switching Latency**: No delay from pause/resume transitions
3. **Natural Interruption**: Users can interrupt TTS playback naturally
4. **Hardware Acceleration**: Leverages macOS optimized audio processing
5. **Integrated VAD**: Built-in voice activity detection reduces CPU load

## Architecture

### Components

1. **Microphone** (`microphone_input.py`)
   - Dynamic node that captures raw audio input at 16kHz
   - Uses sounddevice for audio capture
   - Provides near-end signal to AEC processor

2. **AEC Processor** (`dora-aec`)
   - Receives both microphone input and TTS reference
   - Performs echo cancellation using macOS VoiceProcessingIO
   - Outputs clean audio even during TTS playback
   - Provides hardware-accelerated voice activity detection

3. **Speech Monitor** (`dora-speechmonitor`)
   - Monitors speech patterns without pause/resume logic
   - Detects question completion through silence analysis
   - Receives echo-cancelled audio from AEC processor

4. **Chat Controller AEC** (`dora-chat-controller-aec`)
   - Simplified orchestration without pause/resume state machine
   - Manages conversation flow and history
   - Handles question detection and LLM routing

5. **Audio Player AEC** (`audio_player_aec.py`)
   - Simplified player without buffer status reporting
   - Direct audio playback without feedback loop
   - Reduced complexity and latency

### Data Flow

```
Microphone (dynamic) ───────┐
                           ▼
                      AEC Processor ◄──── PrimeSpeech (reference)
                           │                    ▲
                           ▼                    │
                    Speech Monitor          Text Segmenter
                           │                    ▲
                           ▼                    │
                         ASR                 Qwen3 LLM
                           │                    ▲
                           ▼                    │
                   Chat Controller ─────────────┘
                           │
                           ▼
                    Audio Player
```

The key insight is that the AEC Processor receives:
1. **Near-end signal**: Raw microphone input (speech + echo)
2. **Far-end signal**: TTS output as reference for echo cancellation

## Installation

1. Install the AEC node:
```bash
pip install -e ../../node-hub/dora-aec
```

2. Install the simplified chat controller:
```bash
pip install -e ../../node-hub/dora-chat-controller-aec
```

3. Copy the native library (if not already present):
```bash
# The libAudioCapture.dylib should be in one of:
# - ../../node-hub/dora-aec/dora_aec/lib/
# - ~/home/VoiceDialogue/libraries/
```

## Usage

Start the voice chatbot with AEC:

```bash
dora up
dora start voice_chatbot_aec.yml
```

## Configuration

### AEC Microphone Settings

```yaml
env:
  SAMPLE_RATE: 16000      # Must match ASR requirements
  AEC_ENABLED: true       # Enable echo cancellation
  VAD_ENABLED: true       # Enable voice activity detection
  AUTO_START: true        # Start capture automatically
```

### Chat Controller Settings

```yaml
env:
  NODE_TIMEOUT_MS: 1000        # Event processing timeout
  MAX_HISTORY_LENGTH: 10       # Conversation history size
```

## How AEC Works

1. **Signal Separation**: 
   - Near-end signal: Microphone input (user speech + echo)
   - Far-end signal: TTS output (reference for echo removal)

2. **Echo Path Modeling**: The VoiceProcessingIO AudioUnit models the acoustic path from speaker to microphone

3. **Adaptive Filtering**: Continuously adapts to remove the far-end signal (TTS) from the near-end signal (microphone)

4. **Residual Suppression**: Removes any remaining echo using spectral subtraction

5. **VAD Integration**: Hardware VAD provides accurate voice detection even with background TTS

## Comparison: AEC vs Pause/Resume

| Feature | Pause/Resume | AEC |
|---------|-------------|-----|
| Can interrupt TTS | No | Yes |
| Switching latency | 1-2 seconds | None |
| Implementation complexity | High (state machine) | Low (automatic) |
| CPU usage | Low | Low (hardware accelerated) |
| Natural conversation | Limited | Full-duplex |
| Echo handling | Avoided | Cancelled |

## Troubleshooting

### No Audio Input

1. Check microphone permissions for the terminal/IDE
2. Verify libAudioCapture.dylib is accessible
3. Ensure AEC node started successfully (check logs)

### Echo Still Present

1. Verify `AEC_ENABLED: true` in configuration
2. Check audio input/output devices in macOS settings
3. Ensure speaker volume isn't too high (may saturate AEC)

### High Latency

1. Reduce `NODE_TIMEOUT_MS` if needed
2. Check network latency to LLM service
3. Monitor CPU usage for bottlenecks

## Technical Details

### Native Library Interface

The AEC implementation uses a Swift-based native library that exposes:

```c
void startRecord();
void stopRecord();
uint8_t* getAudioData(int* size, bool* isVoiceActive);
void freeAudioData(uint8_t* buffer);
```

### Audio Format

- **Input**: 16kHz, mono, 16-bit signed integer
- **Output**: 22.05kHz, mono, float32 (for TTS)
- **Processing**: Real-time with <10ms latency

### Memory Management

- Native library allocates audio buffers
- Python wrapper ensures proper deallocation
- No memory leaks with proper cleanup

## Future Enhancements

1. **Reference Signal Input**: Feed TTS output directly to AEC for better cancellation
2. **Multi-Channel Support**: Stereo or spatial audio processing
3. **Noise Suppression**: Additional background noise removal
4. **AGC Integration**: Automatic gain control for consistent levels
5. **Cross-Platform**: Extend beyond macOS to Windows/Linux

## License

Apache-2.0