# 🚀 Voice Assistant Usage Guide

A complete guide to running the macOS voice assistant pipeline with acoustic echo cancellation, speech recognition, LLM processing, and text-to-speech.

## Table of Contents
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [Configuration](#configuration)
- [Running the Pipeline](#running-the-pipeline)
- [Troubleshooting](#troubleshooting)
- [Advanced Usage](#advanced-usage)

## Prerequisites

### System Requirements
- **macOS** (required for MAC-AEC)
- **Apple Silicon** (M1/M2/M3) recommended for best performance
- **16GB RAM** minimum (32GB recommended for larger models)
- **50GB free disk space** for models

### Software Requirements
- Python 3.8+
- Dora framework installed
- Microphone and speakers

## Installation

### 1. Install Dora Nodes

```bash
# Navigate to the example directory
cd /Users/yuechen/home/conversation/dora/examples/mac-aec-chat

# Install ASR node
pip install -e ../../node-hub/dora-asr

# Install LLM node
pip install -e ../../node-hub/dora-qwen3

# Install Text Segmenter
pip install -e ../../node-hub/dora-text-segmenter

# Install TTS node
pip install -e ../../node-hub/dora-primespeech

# Install Conversation Controller (if needed)
pip install -e ../../node-hub/dora-conversation-controller
```

### 2. Download Required Models

Use the model manager to download necessary models:

```bash
cd ../model-manager

# Download ASR model (choose one)
python download_models.py --download funasr-paraformer  # For Chinese
python download_models.py --download whisper-large-v3   # For multilingual

# Download LLM model (choose based on your RAM)
python download_models.py --download qwen3-8b-mlx-4bit   # 16GB RAM
python download_models.py --download qwen3-14b-mlx-4bit  # 24GB RAM
python download_models.py --download qwen3-32b-mlx-6bit  # 32GB+ RAM

# Download TTS model (if using PrimeSpeech)
python download_models.py --download primespeech-base
```

### 3. Verify Installation

```bash
# Check if all nodes are installed
python -c "import dora_asr, dora_qwen3, dora_primespeech; print('All nodes installed!')"

# List cached models
python ../model-manager/download_models.py --list
```

## Quick Start

### Basic Run

```bash
# Run the voice assistant
dora run voice-chat-with-aec.yml
```

### What to Expect

1. **Startup** (10-30 seconds)
   - Models loading into memory
   - Nodes initializing
   - Audio devices connecting

2. **Ready State**
   - You'll see: "Ready for chat!" in the logs
   - The system is now listening

3. **Interaction**
   - Speak naturally into your microphone
   - Wait for the beep or visual indicator
   - The assistant will respond through your speakers

4. **Shutdown**
   - Press `Ctrl+C` to stop
   - Wait for graceful shutdown

## Configuration

### Edit Configuration File

Open `voice-chat-with-aec.yml` and modify environment variables:

#### ASR Configuration
```yaml
env:
  ASR_ENGINE: funasr  # or 'whisper'
  LANGUAGE: zh        # zh, en, or auto
  WHISPER_MODEL: large  # tiny, base, small, medium, large
  ENABLE_PUNCTUATION: true
```

#### LLM Configuration
```yaml
env:
  MLX_MODEL: "Qwen/Qwen3-32B-MLX-6bit"  # Your chosen model
  MAX_TOKENS: 256                        # Response length
  TEMPERATURE: 0.7                       # Creativity (0.0-1.0)
  HISTORY_STRATEGY: "token_based"       # or "fixed"
  MAX_HISTORY_TOKENS: "3000"            # Context window
```

#### TTS Configuration
```yaml
env:
  VOICE_NAME: Doubao  # Available: Doubao, Luo Xiang, Yang Mi, etc.
  TEXT_LANG: zh       # zh or en
  SPEED_FACTOR: 1.0   # Speech speed (0.5-2.0)
```

### System Prompt Customization

Modify the `SYSTEM_PROMPT` to change the assistant's behavior:

```yaml
SYSTEM_PROMPT: "你是一个友好的AI助手。回答简洁明了，每次回复控制在2-3句话。"
```

## Running the Pipeline

### Standard Operation

```bash
# Run with default settings
dora run voice-chat-with-aec.yml

# Run with custom log level
LOG_LEVEL=DEBUG dora run voice-chat-with-aec.yml

# Run with specific model
MLX_MODEL="Qwen/Qwen3-14B-MLX-4bit" dora run voice-chat-with-aec.yml
```

### Monitoring Output

Watch for these key messages:

```
[INFO] Loading MLX model: Qwen/Qwen3-32B-MLX-6bit
[INFO] Ready for chat!
[INFO] History: Token-based strategy - max 3000 tokens
[INFO] Speech detected, recording...
[INFO] Transcription: "你好，今天天气怎么样？"
[INFO] Response in 1.2s | Tokens saved: 245
[INFO] TTS: Synthesizing segment 1/3
```

### Performance Monitoring

Key metrics to watch:
- **Response time**: Should be 1-3 seconds
- **Tokens/second**: 20-50 for good performance
- **Audio latency**: <500ms for smooth interaction

## Troubleshooting

### Common Issues

#### 1. "Model not found"
```bash
# Download the missing model
cd ../model-manager
python download_models.py --download qwen3-32b-mlx-6bit
```

#### 2. "Microphone permission denied"
- Go to System Settings → Privacy & Security → Microphone
- Allow Terminal/Dora access

#### 3. "Out of memory"
- Use a smaller model:
```yaml
MLX_MODEL: "Qwen/Qwen3-8B-MLX-4bit"  # Instead of 32B
```

#### 4. "No audio output"
```bash
# Check audio devices
system_profiler SPAudioDataType

# Test audio player
dora run test_audio_player.yml
```

#### 5. "ASR not detecting speech"
- Check microphone volume
- Verify language setting matches your speech
- Try different ASR engine (whisper vs funasr)

### Debug Mode

Enable detailed logging:

```yaml
env:
  LOG_LEVEL: DEBUG
  ASR_DEBUG: true
  LLM_DEBUG: true
```

### Process Management

```bash
# Kill all Dora processes
dora destroy

# Clean up resources
dora stop

# Check running nodes
dora list
```

## Advanced Usage

### Custom Language Models

1. Download a custom model:
```bash
python ../model-manager/download_models.py --repo mlx-community/Llama-3.2-3B-Instruct-4bit
```

2. Update configuration:
```yaml
MLX_MODEL: "mlx-community/Llama-3.2-3B-Instruct-4bit"
```

### Multi-Language Support

For multilingual conversations:

```yaml
# ASR settings
LANGUAGE: auto
ENABLE_LANGUAGE_DETECTION: true

# LLM prompt
SYSTEM_PROMPT: "You are a multilingual assistant. Respond in the same language as the user's input."

# TTS settings
TEXT_LANG: auto
```

### Performance Optimization

#### For Faster Response
```yaml
MAX_TOKENS: 128          # Shorter responses
TEMPERATURE: 0.5         # Less creative, faster
LLM_ENABLE_STREAMING: true
MAX_HISTORY_EXCHANGES: 3  # Less context
```

#### For Better Quality
```yaml
MAX_TOKENS: 512          # Longer responses
TEMPERATURE: 0.8         # More creative
WHISPER_MODEL: large-v3  # Better ASR
MAX_HISTORY_TOKENS: 5000 # More context
```

### Integration with External Services

Add custom nodes to the pipeline:

```yaml
nodes:
  # ... existing nodes ...
  
  # Add translation node
  - id: translator
    path: custom_translator.py
    inputs:
      text: qwen3-llm/text
    outputs:
      - translated_text
```

### Recording Conversations

Enable conversation logging:

```yaml
# Add a recording node
- id: recorder
  path: conversation_recorder.py
  inputs:
    transcription: asr/transcription
    response: qwen3-llm/text
```

## Best Practices

### 1. Model Selection
- Start with smaller models and upgrade as needed
- Match model size to your hardware capabilities
- Use quantized models for better performance

### 2. Audio Setup
- Use a good quality microphone
- Position microphone away from speakers
- Test echo cancellation with music playing

### 3. Language Configuration
- Set specific language for better accuracy
- Use auto-detection only when necessary
- Match TTS language to LLM output

### 4. Resource Management
- Monitor RAM usage with Activity Monitor
- Close unnecessary applications
- Restart pipeline if performance degrades

## Tips and Tricks

### Quick Commands

```bash
# Quick test with minimal model
MLX_MODEL="Qwen/Qwen3-8B-MLX-4bit" MAX_TOKENS=64 dora run voice-chat-with-aec.yml

# Chinese-only setup
LANGUAGE=zh TEXT_LANG=zh ASR_ENGINE=funasr dora run voice-chat-with-aec.yml

# English-only setup
LANGUAGE=en TEXT_LANG=en WHISPER_MODEL=base dora run voice-chat-with-aec.yml
```

### Useful Aliases

Add to your `.bashrc` or `.zshrc`:

```bash
alias voice-assistant='cd /path/to/mac-aec-chat && dora run voice-chat-with-aec.yml'
alias voice-debug='LOG_LEVEL=DEBUG dora run voice-chat-with-aec.yml'
alias voice-kill='dora destroy'
```

## Support

### Getting Help
- Check logs for error messages
- Enable DEBUG logging for detailed info
- Review ARCHITECTURE.md for system details

### Common Log Locations
- Dora logs: `./out/`
- Node logs: Check terminal output
- System logs: Console.app

### Reporting Issues
When reporting issues, include:
1. System info (macOS version, hardware)
2. Model configuration
3. Error messages from logs
4. Steps to reproduce

## Next Steps

1. **Customize the system prompt** for your use case
2. **Try different voices** in TTS settings
3. **Experiment with models** for optimal performance
4. **Add custom nodes** for additional features
5. **Integrate with your applications**

---

For technical details, see [ARCHITECTURE.md](ARCHITECTURE.md).
For model management, see [../model-manager/README.md](../model-manager/README.md).