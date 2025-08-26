# Voice Chatbot Example

A complete real-time voice conversation system built with Dora. Features automatic speech recognition, LLM processing with streaming support, and text-to-speech synthesis.

## Quick Start Guide

**Prerequisites:** Have headphones ready to prevent audio feedback.

```bash
# Terminal 1: Start dataflow
cd examples/voice-chatbot
dora start voice-chat-no-aec.yml --name voice-chat --attach

# Terminal 2: Audio player
python audio_player.py

# Terminal 3: Microphone
python microphone_input.py

# Terminal 4: Viewer (optional)
python viewer.py
```

Then speak naturally and wait 1.5 seconds after finishing. The AI will respond through your headphones.

## Overview

This example provides two voice chatbot configurations:
1. **voice-chat-no-aec.yml** - Software-based VAD without native AEC (NEW)
2. **voice_chatbot.yml** - Full-duplex with chat controller (original)

The new configuration offers a simpler, more streamlined pipeline that directly connects ASR → LLM → TTS without a chat controller, leveraging the latest streaming capabilities.

## Features

### Common Features
- **Automatic Speech Recognition**: FunASR for Chinese, Whisper for English
- **Streaming LLM**: Direct streaming from Qwen3 to text segmenter
- **Advanced Text Segmentation**: Queue-based segmenter with no deadlocks
- **High-Quality TTS**: PrimeSpeech with multiple voice options
- **Conversation Memory**: Token-based history management
- **Multi-language Support**: Chinese and English

### voice-chat-no-aec.yml (NEW - Recommended)
- **Software VAD**: Robust voice activity detection without hardware dependencies
- **Direct Pipeline**: ASR → LLM → Text Segmenter → TTS (no chat controller)
- **Streaming Support**: LLM streams directly to text segmenter
- **Simplified Architecture**: Fewer components, easier to debug
- **No AEC Required**: Works on any system without native AEC support

### voice_chatbot.yml (Original)
- **Chat Controller**: Central orchestration with pause/resume
- **Hardware AEC**: Requires native acoustic echo cancellation
- **Full-Duplex**: Prevents audio feedback with microphone control
- **Complex Flow**: More components for advanced control

## Architecture

### voice-chat-no-aec.yml (Simplified Pipeline)
```
Microphone → Speech Monitor (VAD) → ASR → Qwen3 LLM
                                            ↓ (streaming)
Audio Player ← PrimeSpeech TTS ← Text Segmenter
```

Key improvements:
- Direct ASR to LLM connection
- Streaming from LLM to text segmenter
- No chat controller needed
- Software-based VAD in speech monitor

### voice_chatbot.yml (Original with Chat Controller)
```
Microphone → Speech Monitor → ASR → Chat Controller
                ↑                           ↓ question
            pause/resume                Qwen3 LLM
                                           ↓ text
Audio Player ← PrimeSpeech TTS ← Text Segmenter
        ↓ buffer_status
    Chat Controller (monitors completion)
```

## Key Components

### 1. Speech Monitor
- Software-based VAD for voice detection
- Configurable silence thresholds for turn-taking
- Audio segmentation based on speech patterns
- Optional pause/resume capability (for chat controller mode)

### 2. Text Segmenter (Updated)
- Queue-based implementation with no deadlocks
- Supports streaming LLM input
- Sends first segment immediately
- Filters punctuation-only segments
- Backpressure control with TTS

### 3. Direct LLM Streaming (NEW)
- LLM streams directly to text segmenter
- No intermediate chat controller needed
- Faster response times
- Simpler error handling

### 4. Chat Controller (Optional - voice_chatbot.yml only)
- Central orchestration for complex scenarios
- Pause/resume microphone control
- Buffer monitoring for completion detection

## Installation

### Prerequisites

1. Install required node packages:
```bash
# Core components (required for both configurations)
cd ../../node-hub/dora-speechmonitor
pip install -e .

cd ../dora-asr
pip install -e .

cd ../dora-qwen3
pip install -e .

cd ../dora-text-segmenter
pip install -e .

cd ../dora-primespeech
pip install -e .

# Optional: Chat Controller (only for voice_chatbot.yml)
cd ../dora-chat-controller
pip install -e .

cd ../dora-conversation-controller
pip install -e .

# Return to voice-chatbot
cd ../../examples/voice-chatbot
```

2. Download models (if needed):
```bash
# Whisper models download automatically

# Qwen3 models:
# For Apple Silicon - MLX model will be downloaded automatically
# For other systems - GGUF model will be downloaded automatically

# FunASR models (optional, for better Chinese ASR):
cd ../dora-asr-test
./download_funasr_models.sh
cd ../voice-chatbot
```

## Usage

### Quick Start

#### For voice-chat-no-aec.yml (Recommended)

**Important**: Use headphones to prevent echo/feedback issues. Without headphones, the microphone may pick up the AI's voice causing audio loops.

**Step-by-step startup:**

1. **Terminal 1 - Start the dataflow:**
```bash
cd examples/voice-chatbot
dora start voice-chat-no-aec.yml --name voice-chat --attach
```
Wait for "All nodes are now running" message.

2. **Terminal 2 - Start audio player:**
```bash
cd examples/voice-chatbot
python audio_player.py
```
You should see: "Audio player ready. Waiting for audio..."

3. **Terminal 3 - Start microphone input:**
```bash
cd examples/voice-chatbot
python microphone_input.py
```
You should see: "Microphone input started. Speak into your microphone..."

4. **Terminal 4 - Start viewer (optional but recommended):**
```bash
cd examples/voice-chatbot
python viewer.py
```
This shows transcriptions and AI responses in real-time.

**To test:** Say something like "Hello, how are you?" and wait 1.5 seconds. You should hear the AI respond through your headphones.

**To stop:** Press Ctrl+C in each terminal, starting with the microphone input, then audio player, then viewer, and finally the dataflow.

#### For voice_chatbot.yml (Original)
```bash
# Run with defaults
./run_chatbot.sh
```

### Custom Configuration

```bash
# Adjust silence detection (when question ends)
USER_SILENCE_THRESHOLD_MS=2000 ./run_chatbot.sh  # 2 seconds

# Adjust conversation parameters
MIN_QUESTION_LENGTH=10 MAX_CONVERSATION_HISTORY=20 ./run_chatbot.sh

# Adjust answer completion detection
ANSWER_COMPLETE_BUFFER_THRESHOLD=3.0 ./run_chatbot.sh
```

### Manual Start

Terminal 1 - Start dataflow:
```bash
dora start voice_chatbot.yml --name voice-chatbot --attach
```

Terminal 2 - Start microphone:
```bash
python microphone_input.py
```

Terminal 3 - Start audio player:
```bash
python audio_player.py
```

Terminal 4 - Start chat controller:
```bash
python chat_controller.py
```

## Configuration

### Environment Variables

**Speech Detection (Both Configurations):**
- `USER_SILENCE_THRESHOLD_MS` - Silence duration to end question (default: 1500ms)
- `SILENCE_THRESHOLD_MS` - Internal silence threshold (default: 500ms)
- `VAD_THRESHOLD` - Voice activity detection sensitivity (default: 0.6)
- `VAD_ENABLED` - Enable software VAD (default: true)

**Chat Controller (voice_chatbot.yml only):**
- `MIN_QUESTION_LENGTH` - Minimum characters for valid question (default: 5)
- `MAX_CONVERSATION_HISTORY` - Number of Q&A pairs to remember (default: 10)
- `ANSWER_COMPLETE_BUFFER_THRESHOLD` - Buffer % to consider answer complete (default: 5.0)

**LLM (Qwen3):**
- `USE_MLX` - Enable MLX for Apple Silicon (default: auto)
- `MLX_MODEL` - MLX model identifier (default: Qwen/Qwen3-8B-MLX-4bit)
- `MAX_TOKENS` - Max response length (default: 256)
- `TEMPERATURE` - Response creativity (default: 0.7)
- `LLM_ENABLE_STREAMING` - Enable streaming to text segmenter (default: true)
- `HISTORY_STRATEGY` - History management: "token_based", "fixed", or "sliding_window"
- `SYSTEM_PROMPT` - Configures assistant behavior for voice interaction

**TTS (PrimeSpeech):**
- `VOICE_NAME` - Voice selection: Doubao, Luo Xiang, Yang Mi, Zhou Jielun, Ma Yun, Maple, Cove
- `ENABLE_INTERNAL_SEGMENTATION` - Split long text internally (default: true)
- `TTS_MAX_SEGMENT_LENGTH` - Max chars per TTS segment (default: 100)
- `SPEED_FACTOR` - Speech speed multiplier (default: 1.0)

**Text Segmenter:**
- `ENABLE_BACKPRESSURE` - Wait for TTS completion (default: false for first segment)
- `SEGMENT_MODE` - Segmentation mode: "sentence", "punctuation", or "fixed"
- `MIN_SEGMENT_LENGTH` - Minimum segment length (default: 5)
- `MAX_SEGMENT_LENGTH` - Maximum segment length (default: 20)

## How It Works

### voice-chat-no-aec.yml Flow (Simplified)

1. **Voice Detection**
   - Software VAD detects speech in microphone input
   - Audio segments sent to ASR when speech ends

2. **Transcription & Processing**
   - ASR transcribes audio to text
   - Text directly sent to Qwen3 LLM
   - LLM streams response to text segmenter

3. **Speech Synthesis**
   - Text segmenter buffers and segments the streaming text
   - Sends segments to TTS one at a time
   - Audio player outputs synthesized speech

### voice_chatbot.yml Flow (With Chat Controller)

1. **Listening State**
   - Speech Monitor detects speech and sends audio to ASR
   - Chat Controller accumulates transcriptions
   - When speech ends, question is complete

2. **Processing State**
   - Chat Controller pauses Speech Monitor
   - Sends complete question to Qwen3 LLM
   - LLM generates response

3. **Answering State**
   - Answer flows through Text Segmenter → TTS → Audio Player
   - Chat Controller monitors buffer status
   - Resumes listening when playback completes

### Key Improvements in New Configuration

1. **No Chat Controller Needed**: Direct pipeline reduces complexity
2. **Streaming LLM**: Faster first-token response time
3. **Software VAD**: No hardware AEC dependencies
4. **Queue-Based Segmenter**: No deadlocks, immediate first segment
5. **Simplified Error Handling**: Fewer components to debug

## Troubleshooting

### Startup Issues
- **"node not found" error**: Make sure you've installed all required nodes (see Installation section)
- **"file not found" error**: Ensure you're in the `examples/voice-chatbot` directory
- **No audio output**: Check audio player is running and system volume is up
- **No speech detection**: Check microphone permissions and that microphone_input.py is running

### Echo/Feedback Issues
- **Use headphones** - This is the simplest solution to prevent echo
- Without headphones, the microphone may pick up TTS output causing loops
- For speaker use, consider the mac-aec-chat example with hardware AEC

### No Speech Detection
- Check microphone permissions
- Adjust `VAD_THRESHOLD` (lower = more sensitive)
- Check `MIN_AUDIO_AMPLITUDE` setting

### Premature Question Cutoff
- Increase `USER_SILENCE_THRESHOLD_MS`
- Default 1500ms, try 2000-3000ms for slower speakers

### Answer Cut Off
- Increase `DEFAULT_MAX_TOKENS` for longer responses
- Check `MAX_SEGMENT_LENGTH` for TTS segments

### Audio Glitches
- Check buffer thresholds (PAUSE/RESUME)
- Ensure proper audio device configuration

## Choosing Between Configurations

### Use voice-chat-no-aec.yml when:
- You don't have native AEC support
- You want a simpler, more maintainable setup
- You need faster response times
- You're debugging or developing new features

### Use voice_chatbot.yml when:
- You need full-duplex conversation with echo prevention
- You have native AEC hardware support
- You need complex conversation control
- You want centralized orchestration

## Migration Guide

To migrate from voice_chatbot.yml to voice-chat-no-aec.yml:

1. Remove chat controller dependencies
2. Update Speech Monitor to use software VAD
3. Connect ASR directly to LLM
4. Enable LLM streaming (`LLM_ENABLE_STREAMING: "true"`)
5. Use the latest text segmenter (queue-based)

## Examples

### Sample Configuration (voice-chat-no-aec.yml)

```yaml
# Direct pipeline without chat controller
- id: asr
  inputs:
    audio: speech-monitor/audio_segment
  outputs:
    - transcription

- id: qwen3-llm
  inputs:
    text: asr/transcription  # Direct connection
  env:
    LLM_ENABLE_STREAMING: "true"  # Enable streaming
    
- id: text-segmenter
  inputs:
    text: qwen3-llm/text  # Receives streaming text
```

### Sample Conversation

```
User: "What's the weather like today?"
[1.5s silence - VAD detects end of speech]
[ASR transcribes and sends to LLM]
[LLM starts streaming response]
[Text segmenter sends first segment immediately to TTS]
AI: "I don't have access to real-time weather data, but you can check..."
[Continuous playback as segments arrive]
```

## Performance Comparison

| Aspect | voice-chat-no-aec.yml | voice_chatbot.yml |
|--------|----------------------|-------------------|
| First response time | ~1.5s | ~2.5s |
| Pipeline complexity | Simple (5 nodes) | Complex (7 nodes) |
| AEC support | Software VAD | Hardware AEC required |
| Streaming | Yes | No |
| Chat controller | No | Yes |
| Echo prevention | Basic (use headphones) | Advanced (hardware AEC) |
| Setup difficulty | Easy | Moderate |
| Recommended audio | Headphones | Speakers or headphones |

## Next Steps

1. Try the simplified `voice-chat-no-aec.yml` configuration first
2. Experiment with different TTS voices in PrimeSpeech
3. Adjust VAD sensitivity for your environment
4. Fine-tune the system prompt for your use case
5. Consider adding custom wake word detection

## Related Examples

- **mac-aec-chat**: Hardware AEC with macOS VoiceProcessingIO
- **realtime-interpreter**: Basic ASR to LLM pipeline
- **primespeech-streaming-test**: TTS streaming experiments

## Contributing

Contributions are welcome! Please test with both configurations before submitting PRs.