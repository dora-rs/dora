# Voice Assistant TTS Debug Context

## Current Issue
PrimeSpeech TTS is generating tones instead of real voice output.

## System Architecture

### Pipeline Flow
```
Microphone → Speech Monitor (VAD) → ASR (FunASR) → Qwen3 LLM → Text Segmenter → PrimeSpeech TTS → Audio Player
```

### Current Configuration (voice-chat-no-aec.yml)
- **LLM**: GLM-4.5-Air-3bit (mlx-community)
- **ASR**: FunASR (Chinese)
- **TTS**: PrimeSpeech with Doubao voice
- **System Prompt**: Chinese, non-thinking mode

## PrimeSpeech TTS Setup

### Model Locations
```bash
# Downloaded models should be in:
~/.dora/models/primespeech/moyoyo/
├── GPT_weights/
│   └── doubao_best_gpt.ckpt        # Should be ~140MB
├── SoVITS_weights/
│   └── doubao_best_sovits.pth      # Should be ~80MB
├── ref_audios/
│   └── doubao_ref.wav              # Should be >50KB
├── chinese-hubert-base/
│   └── pytorch_model.bin           # Should be ~180MB
└── chinese-roberta-wwm-ext-large/
    └── pytorch_model.bin           # Should be ~610MB
```

### Configuration Files

#### 1. PrimeSpeech Node Configuration (in voice-chat-no-aec.yml)
```yaml
- id: primespeech
  env:
    USE_HUGGINGFACE_MODELS: "false"  # Set to "true" to use downloaded models
    VOICEDIALOGUE_PATH: /Users/yuechen/home/VoiceDialogue  # Local path fallback
    VOICE_NAME: Doubao
    TEXT_LANG: zh
    USE_GPU: false
    LOG_LEVEL: INFO  # Change to DEBUG for more details
```

#### 2. Available Voices
- **Chinese**: Doubao, Luo Xiang, Yang Mi, Zhou Jielun, Ma Yun, BYS, Ma Baoguo, Shen Yi
- **English**: Maple, Cove, Ellen, Juniper, Trump

## Debug Steps

### Step 1: Check Model Files
```bash
# Check if models are real or placeholders
ls -lah ~/.dora/models/primespeech/moyoyo/GPT_weights/
ls -lah ~/.dora/models/primespeech/moyoyo/SoVITS_weights/

# If files are <1MB, they're placeholders. Download real models:
cd /Users/yuechen/home/arios/dora/examples/model-manager
./download_models.py --download primespeech-base
./download_models.py --voice Doubao
```

### Step 2: Verify Model Loading
```bash
# Run the pipeline with debug logging
cd /Users/yuechen/home/arios/dora
export LOG_LEVEL=DEBUG
dora start examples/voice-chatbot/voice-chat-no-aec.yml
```

### Step 3: Test TTS Independently
Create `test_tts.py`:
```python
#!/usr/bin/env python3
import sys
from pathlib import Path
import numpy as np
import soundfile as sf

# Add PrimeSpeech to path
sys.path.append("/Users/yuechen/home/arios/dora/node-hub/dora-primespeech")

from dora_primespeech.config import VOICE_CONFIGS
from dora_primespeech.moyoyo_tts_wrapper_streaming_fix import MoYoYoTTSWrapper

# Test TTS
models_path = Path.home() / ".dora" / "models" / "primespeech" / "moyoyo"
print(f"Using models from: {models_path}")
print(f"Models exist: {models_path.exists()}")

tts = MoYoYoTTSWrapper(
    voice="doubao",
    device="cpu",
    models_path=models_path,
    voice_config=VOICE_CONFIGS["Doubao"]
)

# Test synthesis
text = "你好，这是一个测试。如果你能听到清晰的声音，说明TTS工作正常。"
print(f"Synthesizing: {text}")

sample_rate, audio = tts.synthesize(text, language="zh", speed=1.0)
print(f"Sample rate: {sample_rate}, Audio shape: {audio.shape}")

# Save to file
sf.write("test_output.wav", audio, sample_rate)
print("Saved to test_output.wav - check if it's real voice or just tones")
```

### Step 4: Check Configuration Priority
The TTS checks paths in this order:
1. `models_path` parameter (if provided)
2. `VOICEDIALOGUE_PATH` environment variable
3. Default: `~/VoiceDialogue` or `/Users/yuechen/home/VoiceDialogue`

To force using downloaded models:
```yaml
USE_HUGGINGFACE_MODELS: "true"  # Must set this!
```

## Recent Fixes Applied

### 1. Fixed Hardcoded Path Issue
- **File**: `moyoyo_tts_wrapper_streaming_fix.py`
- **Fix**: Added configurable path via environment variable
- **Commit**: 27e97ad1

### 2. Added Voice Configuration
- **File**: `config.py`
- **Fix**: Added all 13 voice configurations
- **Commit**: 19013967

### 3. Model Manager Improvements
- **File**: `model_manager.py`
- **Fix**: Added proper download functionality
- **Commit**: 19013967, c4002f22

## Quick Diagnosis Commands

```bash
# 1. Check what models are downloaded
cd /Users/yuechen/home/arios/dora/examples/model-manager
./download_models.py --list | grep primespeech

# 2. Check model sizes (should be >100MB each)
du -sh ~/.dora/models/primespeech/moyoyo/*

# 3. Check if placeholder files
file ~/.dora/models/primespeech/moyoyo/GPT_weights/*.ckpt
# Should show "data" not "ASCII text"

# 4. Re-download if needed
./download_models.py --voice Doubao --models-dir ~/.dora/models/primespeech

# 5. Test with HuggingFace models enabled
sed -i '' 's/USE_HUGGINGFACE_MODELS: "false"/USE_HUGGINGFACE_MODELS: "true"/' \
  examples/voice-chatbot/voice-chat-no-aec.yml
```

## Common Issues and Solutions

| Issue | Symptom | Solution |
|-------|---------|----------|
| Placeholder models | Files <1MB, ASCII text | Download real models with `download_models.py` |
| Wrong path | Using VoiceDialogue instead of downloaded | Set `USE_HUGGINGFACE_MODELS: "true"` |
| Model load failure | Silent failure, outputs tones | Check logs with `LOG_LEVEL: DEBUG` |
| Missing base models | chinese-hubert-base not found | Run `download_models.py --download primespeech-base` |

## Environment Variables to Set

```bash
export PRIMESPEECH_MODEL_DIR=~/.dora/models/primespeech
export USE_HUGGINGFACE_MODELS=true
export LOG_LEVEL=DEBUG
```

## Files to Monitor

1. **Main TTS Wrapper**: `/Users/yuechen/home/arios/dora/node-hub/dora-primespeech/dora_primespeech/moyoyo_tts_wrapper_streaming_fix.py`
2. **Configuration**: `/Users/yuechen/home/arios/dora/node-hub/dora-primespeech/dora_primespeech/config.py`
3. **Model Manager**: `/Users/yuechen/home/arios/dora/node-hub/dora-primespeech/dora_primespeech/model_manager.py`
4. **Pipeline Config**: `/Users/yuechen/home/arios/dora/examples/voice-chatbot/voice-chat-no-aec.yml`

## Root Cause Analysis: Tone Generation

### When Tones Are Generated

The 440Hz sine wave tone is generated in `moyoyo_tts_wrapper_streaming_fix.py` under these conditions:

```python
def synthesize_streaming(self, text, language="zh", speed=1.0):
    if not MOYOYO_AVAILABLE or self.tts is None:
        # GENERATES TONE!
        audio = np.sin(2 * np.pi * 440 * t) * 0.3
```

**Triggers:**
1. `MOYOYO_AVAILABLE = False` - MoYoYo module import failed
2. `self.tts = None` - TTS initialization failed
3. Any exception during synthesis

### Common Causes

1. **Placeholder Model Files**
   - Files < 1MB containing text like "placeholder" or "download required"
   - Solution: Download real models with model-manager

2. **Wrong Path Configuration**
   - Using hardcoded VoiceDialogue path instead of downloaded models
   - Solution: Set `USE_HUGGINGFACE_MODELS: "true"`

3. **Missing Base Models**
   - chinese-hubert-base or chinese-roberta-wwm-ext-large not found
   - Solution: Run `download_models.py --download primespeech-base`

4. **Import Failures**
   - MoYoYo dependencies not installed
   - Solution: Install dora-primespeech package

## Diagnostic Tools

### 1. Diagnose Issues
```bash
python examples/voice-chatbot/diagnose_tts.py
```
This script checks:
- Model file sizes and validity
- Environment configuration
- Python module imports
- Provides specific fixes

### 2. Test TTS
```bash
python examples/voice-chatbot/test_primespeech.py
```
This script:
- Tests actual TTS synthesis
- Detects if output is tone vs real voice
- Saves test audio for verification

## Next Steps

1. **Run diagnostics** to identify specific issues:
   ```bash
   python examples/voice-chatbot/diagnose_tts.py
   ```

2. **Fix identified issues** (usually downloading models):
   ```bash
   cd examples/model-manager
   python download_models.py --download primespeech-base
   python download_models.py --voice Doubao
   ```

3. **Enable HuggingFace models** in configuration:
   ```yaml
   USE_HUGGINGFACE_MODELS: "true"
   ```

4. **Test TTS** to verify fix:
   ```bash
   python examples/voice-chatbot/test_primespeech.py
   ```

5. **Run pipeline** with working TTS:
   ```bash
   dora start examples/voice-chatbot/voice-chat-no-aec.yml
   ```

## Contact Points

- Repository: MoYoYoTech/tone-models (HuggingFace)
- Local backup: /Users/yuechen/home/VoiceDialogue (if available)
- Model cache: ~/.dora/models/primespeech/

---
Generated: 2024-08-26
Context for: TTS generating tones instead of voice issue