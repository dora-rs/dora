# Voice Chatbot Example

A complete real-time voice conversation system built with Dora. Features full-duplex conversation with automatic speech recognition, LLM processing, and text-to-speech synthesis.

## Overview

This example demonstrates a production-ready voice chatbot that enables natural conversations with an AI assistant. The system automatically handles turn-taking, prevents audio feedback, manages buffer flow, and maintains conversation context.

## Features

- **Full-Duplex Conversation**: Seamless switching between listening and speaking
- **Automatic Turn-Taking**: Detects when user finishes speaking (3s silence)
- **Echo Prevention**: Pauses microphone during AI responses
- **Backpressure Control**: Prevents buffer overflow with flow control
- **Conversation Memory**: Maintains context across multiple exchanges
- **Multi-language Support**: Works with Chinese and English
- **Real-time Processing**: Low-latency response (< 2 seconds)
- 📝 **Context Awareness** - Maintains conversation history for context-aware responses

## Architecture

This system combines components from:
- **realtime-interpreter** example (microphone, speech monitor, ASR, Qwen3)
- **primespeech-streaming-test** example (text segmenter, TTS, audio player, flow control)
- **NEW: Chat Controller** - Central orchestrator managing conversation flow

```
Microphone → Speech Monitor → ASR → Chat Controller
                ↑                           ↓ question
            pause/resume                Qwen3 LLM
                                           ↓ text (direct)
Audio Player ← PrimeSpeech TTS ← Text Segmenter
        ↓ buffer_status
    Chat Controller (monitors completion)
```

## Key Components

### 1. Modified Speech Monitor
- Added pause/resume capability via control input
- Drops all audio when paused (clean silence)
- Resumes cleanly without audio artifacts

### 2. Chat Controller (New)
- Accumulates transcriptions until speech ends
- Detects question completion using Speech Monitor's silence detection
- Pauses microphone input during answer generation/playback
- Monitors audio buffer to detect answer completion
- Maintains conversation history for context
- Works with Qwen3's system prompt for consistent behavior

### 3. Integrated Pipeline
- Seamless handoff between input and output pipelines
- Backpressure control prevents buffer overflow
- One-segment-at-a-time TTS processing

## Installation

### Prerequisites

1. Install required node packages:
```bash
# Speech Monitor
cd ../../node-hub/dora-speechmonitor
pip install -e .

# ASR
cd ../dora-asr
pip install -e .

# Qwen3 LLM
cd ../dora-qwen3
pip install -e .

# Chat Controller (Static Node)
cd ../dora-chat-controller
pip install -e .

# Text Segmenter
cd ../dora-text-segmenter
pip install -e .

# Conversation Controller
cd ../dora-conversation-controller
pip install -e .

# PrimeSpeech TTS
cd ../dora-primespeech
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

**Speech Detection:**
- `USER_SILENCE_THRESHOLD_MS` - Silence duration to end question (default: 1500ms)
- `SILENCE_THRESHOLD_MS` - Internal silence threshold (default: 500ms)
- `VAD_THRESHOLD` - Voice activity detection sensitivity (default: 0.6)

**Chat Controller:**
- `MIN_QUESTION_LENGTH` - Minimum characters for valid question (default: 5)
- `MAX_CONVERSATION_HISTORY` - Number of Q&A pairs to remember (default: 10)
- `ANSWER_COMPLETE_BUFFER_THRESHOLD` - Buffer % to consider answer complete (default: 5.0)

**LLM (Qwen3 with MLX):**
- `USE_MLX` - Enable MLX for Apple Silicon (default: auto)
- `MLX_MODEL` - MLX model identifier (default: Qwen/Qwen3-8B-MLX-4bit)
- `GGUF_MODEL` - GGUF model for non-Apple Silicon (default: Qwen/Qwen3-8B-GGUF)
- `MAX_TOKENS` - Max response length (default: 256)
- `TEMPERATURE` - Response creativity (default: 0.7)
- `SYSTEM_PROMPT` - Configures assistant behavior for voice interaction

**TTS:**
- `VOICE_NAME` - TTS voice selection (default: Zhiyu for Chinese)
- `MAX_SEGMENT_LENGTH` - Max chars per TTS segment (default: 50)

**Flow Control:**
- `PAUSE_THRESHOLD` - Buffer % to pause generation (default: 70)
- `RESUME_THRESHOLD` - Buffer % to resume generation (default: 50)

## How It Works

### Conversation Flow

1. **Listening State**
   - Speech Monitor detects speech and sends audio to ASR
   - Chat Controller accumulates transcriptions
   - When speech ends (silence detected), question is complete

2. **Processing State**
   - Chat Controller pauses Speech Monitor
   - Sends complete question to Qwen3 LLM with context
   - LLM generates response

3. **Answering State**
   - Answer sent to Text Segmenter → TTS → Audio Player
   - Chat Controller monitors buffer status
   - When buffer empty (< 5%), answer is complete

4. **Resume Listening**
   - Chat Controller resumes Speech Monitor
   - Ready for next question
   - Conversation history maintained

### Key Innovations

1. **Clean Pause/Resume**: Speech Monitor drops audio when paused, preventing queue buildup
2. **Question Completion**: Uses existing silence detection, no modifications needed
3. **Answer Completion**: Monitors audio buffer to detect playback end
4. **Context Management**: Maintains conversation history for natural dialogue
5. **MLX Optimization**: Uses Qwen3 with MLX for fast inference on Apple Silicon
6. **Voice-Optimized Prompt**: System prompt configured for concise, TTS-friendly responses

## Troubleshooting

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

## Examples

### System Prompt

The Qwen3 LLM is configured with a voice-optimized system prompt:
```
You are a helpful voice assistant. Provide natural, conversational responses 
that are concise and suitable for text-to-speech. Keep responses brief but 
informative, ideally 1-3 sentences unless more detail is specifically requested.
```

This ensures:
- Responses are optimized for TTS playback
- Answers are concise to minimize wait time
- Natural conversational tone
- Appropriate length for voice interaction

### Sample Conversation

```
User: "What's the weather like today?"
[1.5s silence - question complete]
[Speech Monitor paused]