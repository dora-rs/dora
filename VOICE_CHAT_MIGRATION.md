# Voice Chat System Migration to Arios

## Date: August 27, 2024

## Summary
Successfully migrated the complete voice chat system from conversation/dora to arios/dora, including all necessary nodes and examples.

## Copied Components

### 1. Core Nodes (`/node-hub/`)
All nodes copied to `/Users/yuechen/home/arios/dora/node-hub/`:

- **dora-primespeech** - Text-to-speech synthesis with MoYoYo TTS (fixed version with bundled moyoyo_tts)
- **dora-mac-aec** - macOS acoustic echo cancellation with Voice Activity Detection
- **dora-speechmonitor** - Real-time speech detection and segmentation
- **dora-text-segmenter** - Queue-based text segmentation for TTS
- **dora-asr** - Automatic speech recognition (FunASR/Whisper)
- **dora-qwen3** - Qwen3 language model for conversational AI
- **dora-microphone** - Audio input capture

### 2. Example Applications (`/examples/`)
Copied to `/Users/yuechen/home/arios/dora/examples/`:

#### voice-chatbot/
Complete voice assistant pipeline without AEC:
- `voice-chat-no-aec.yml` - Main pipeline configuration
- `audio_player.py` - Audio output handling
- `microphone_input.py` - Audio input capture
- `viewer.py` - Real-time visualization
- Test scripts and documentation

## Key Features

### PrimeSpeech Node (Updated)
- ✅ Standalone operation - no VoiceDialogue dependencies
- ✅ Bundled moyoyo_tts module included
- ✅ Uses PRIMESPEECH_MODEL_DIR environment variable
- ✅ Proper error handling (no 440Hz tone fallback)
- ✅ Consistent logging with send_log

### Voice Chat Pipeline
- Real-time speech detection with VAD
- Streaming ASR transcription
- Qwen3 LLM for responses
- Queue-based text segmentation
- High-quality TTS with multiple voices
- Audio playback with buffer management

## Configuration

### Environment Variables
```yaml
# PrimeSpeech TTS
PRIMESPEECH_MODEL_DIR: ~/.dora/models/primespeech

# ASR (optional - defaults to ~/.dora/models/asr)
ASR_MODELS_DIR: ~/.dora/models/asr

# Qwen3 LLM
MLX_MODEL: "mlx-community/GLM-4.5-Air-3bit"  # For Apple Silicon
```

### Model Structure
```
~/.dora/models/
├── primespeech/
│   └── moyoyo/
│       ├── GPT_weights/
│       ├── SoVITS_weights/
│       ├── ref_audios/
│       ├── chinese-hubert-base/
│       └── chinese-roberta-wwm-ext-large/
└── asr/
    └── (FunASR or Whisper models)
```

## Testing

### Test Individual Nodes
```bash
# Test PrimeSpeech
cd /Users/yuechen/home/arios/dora/node-hub/dora-primespeech
python test_arios_primespeech.py

# Test Speech Monitor
cd /Users/yuechen/home/arios/dora/node-hub/dora-speechmonitor
python -c "from dora_speechmonitor.vad import SileroVAD; print('VAD OK')"

# Test ASR
cd /Users/yuechen/home/arios/dora/node-hub/dora-asr
python -c "from dora_asr.config import ASRConfig; print('ASR OK')"
```

### Run Voice Chat Pipeline
```bash
cd /Users/yuechen/home/arios/dora/examples/voice-chatbot

# Start dora daemon
dora up

# Run the voice chat
dora start voice-chat-no-aec.yml --attach
```

## Dependencies
All nodes include their required dependencies in their respective `pyproject.toml` files:
- Core: dora-rs, numpy, pyarrow
- Audio: sounddevice, soundfile, librosa
- ML: torch, transformers, mlx (for Apple Silicon)
- TTS: pypinyin, jieba, cn2an
- ASR: funasr, whisper

## Notes
1. The moyoyo_tts module is now bundled with dora-primespeech (no external dependencies)
2. All nodes use environment variables for configuration (no hardcoded paths)
3. The pipeline supports both Chinese and English
4. Optimized for real-time conversation with streaming support

## Migration Status
✅ Complete - All components successfully copied and verified