# 🎙️ macOS Voice Assistant with Echo Cancellation

A production-ready voice assistant pipeline for macOS featuring acoustic echo cancellation, real-time speech recognition, LLM processing, and text-to-speech synthesis.

## Features

- **🔊 Acoustic Echo Cancellation** - Removes speaker feedback for hands-free operation
- **🎯 Real-time ASR** - FunASR for Chinese, Whisper for multilingual support  
- **🤖 LLM Integration** - Qwen3 models optimized for Apple Silicon with MLX
- **🗣️ Natural TTS** - PrimeSpeech with multiple voice options
- **⚡ Streaming Pipeline** - Low-latency response with parallel processing
- **💾 Smart Caching** - Token-based conversation history management

## Quick Start

```bash
# 1. Install dependencies
pip install -e ../../node-hub/dora-aec        # IMPORTANT: Use dora-aec, NOT dora-mac-aec
pip install -e ../../node-hub/dora-asr
pip install -e ../../node-hub/dora-qwen3
pip install -e ../../node-hub/dora-text-segmenter
pip install -e ../../node-hub/dora-primespeech

# 2. Download models (using model manager)
cd ../model-manager
python download_models.py --download qwen3-32b-mlx-6bit
python download_models.py --download funasr-paraformer

# 3. Start the pipeline
cd ../mac-aec-chat
dora start voice-chat-with-aec.yml

# 4. Run dynamic nodes (in separate terminals)
python mac_aec_simple_segmentation.py  # Audio capture with AEC
python viewer.py                       # Optional: Monitor pipeline
```

## Documentation

- **[📖 USAGE_GUIDE.md](USAGE_GUIDE.md)** - Complete setup and usage instructions
- **[🏗️ ARCHITECTURE.md](ARCHITECTURE.md)** - Technical architecture and data flow
- **[⚙️ voice-chat-with-aec.yml](voice-chat-with-aec.yml)** - Main configuration file

## Configuration

Edit `voice-chat-with-aec.yml` to customize:

```yaml
# LLM Model (choose based on RAM)
MLX_MODEL: "Qwen/Qwen3-8B-MLX-4bit"   # 16GB RAM
MLX_MODEL: "Qwen/Qwen3-14B-MLX-4bit"  # 24GB RAM  
MLX_MODEL: "Qwen/Qwen3-32B-MLX-6bit"  # 32GB+ RAM

# Language Settings
LANGUAGE: zh        # ASR: zh/en/auto
TEXT_LANG: zh       # TTS: zh/en

# Voice Selection
VOICE_NAME: Doubao  # Doubao/Luo Xiang/Yang Mi/etc.
```

## System Requirements

- **macOS** (required for MAC-AEC)
- **Apple Silicon** (M1/M2/M3) recommended
- **RAM**: 16GB minimum, 32GB for larger models
- **Storage**: 50GB for models

## Project Structure

```
mac-aec-chat/
├── voice-chat-with-aec.yml        # Main pipeline configuration
├── mac_aec_simple_segmentation.py # Audio capture with VAD
├── audio_player.py                # Audio playback node
├── viewer.py                      # Pipeline monitoring (optional)
├── requirements.txt               # Python dependencies
├── USAGE_GUIDE.md                # Setup and usage instructions
├── ARCHITECTURE.md               # Technical documentation
├── DYNAMIC_NODES.md              # Node implementation details
├── README.md                     # This file
├── out/                          # Output logs
├── scripts/                      # Utility scripts
└── test_audio/                   # Test audio files
```

## Pipeline Overview

```
🎤 Microphone Input
    ↓
🔇 MAC-AEC (Echo Cancellation + VAD)
    ↓
🎯 ASR (Speech Recognition)
    ↓
🤖 Qwen3 LLM (Language Model)
    ↓
📝 Text Segmenter (Chunking)
    ↓
🗣️ PrimeSpeech TTS (Speech Synthesis)
    ↓
🔊 Audio Output
```

## Critical Implementation Notes (Golden Version)

⚠️ **IMPORTANT**: The MAC-AEC implementation has specific requirements:

1. **Use `dora-aec`** library, NOT `dora-mac-aec` (latter lacks proper echo cancellation)
2. **Drain audio buffer completely** - loop `get_audio_data()` until None
3. **Send ALL frames** - not just samples (was losing 97% of audio!)
4. **Poll every 10ms** - not 33ms (to avoid gaps)
5. **Convert format** - int16 bytes → float32 arrays for ASR

## Troubleshooting

| Issue | Solution |
|-------|----------|
| Empty recordings | Check audio buffer is being drained completely in loop |
| Echo not cancelled | Ensure using `dora-aec` library, not `dora-mac-aec` |
| Missing audio | Verify polling at 10ms intervals and sending ALL frames |
| Model not found | Use model-manager to download: `python ../model-manager/download_models.py --download MODEL_NAME` |
| Out of memory | Use smaller model: `MLX_MODEL: "Qwen/Qwen3-8B-MLX-4bit"` |
| No audio | Check microphone permissions in System Settings |
| Slow response | Reduce `MAX_TOKENS` and `MAX_HISTORY_EXCHANGES` |

## Performance

- **First response**: 1-2 seconds
- **ASR latency**: 200-500ms  
- **LLM generation**: 20-50 tokens/sec
- **TTS synthesis**: 2-3x real-time

## Related Projects

- [Model Manager](../model-manager/) - Download and manage AI models
- [Dora Framework](https://github.com/dora-rs/dora) - Dataflow runtime

## License

Part of the Dora ecosystem - see main repository for license details.