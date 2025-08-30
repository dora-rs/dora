# PrimeSpeech Node Changes Documentation

## Summary
Fixed PrimeSpeech TTS node to work standalone without VoiceDialogue dependencies and removed 440Hz tone generation fallback.

## Date: August 27, 2024

## Problems Fixed
1. **Hardcoded VoiceDialogue paths** - Node was looking for models in non-existent paths like `/Users/yuechen/home/VoiceDialogue/`
2. **440Hz tone generation** - Fallback tone generation was playing instead of actual TTS when models failed to load
3. **Missing MoYoYo TTS module** - The TTS engine code wasn't included with the node
4. **Logger inconsistency** - Used both logger and send_log functions inconsistently
5. **Complex model path logic** - Had separate logic for "bundled" vs "HuggingFace" models

## Changes Made

### 1. Removed VoiceDialogue Dependencies
**File**: `dora_primespeech/moyoyo_tts_wrapper_streaming_fix.py`

- **Removed**: All hardcoded VoiceDialogue paths
  ```python
  # REMOVED:
  VOICEDIALOGUE_PATH = None
  for possible_path in [
      "/Users/yuechen/home/VoiceDialogue",
      "/home/yuechen/VoiceDialogue",
      # etc...
  ```

- **Added**: Use only PRIMESPEECH_MODEL_DIR environment variable
  ```python
  # NEW:
  if os.environ.get("PRIMESPEECH_MODEL_DIR"):
      self.models_path = Path(os.path.expanduser(os.environ.get("PRIMESPEECH_MODEL_DIR"))) / "moyoyo"
  else:
      raise RuntimeError("No models path available. Please set PRIMESPEECH_MODEL_DIR environment variable.")
  ```

### 2. Removed 440Hz Tone Generation Fallback
**File**: `dora_primespeech/moyoyo_tts_wrapper_streaming_fix.py`

- **Removed**: All tone generation code
  ```python
  # REMOVED:
  def _generate_tone(duration=1.0, frequency=440, sample_rate=32000):
      """Generate a simple sine wave tone"""
      t = np.linspace(0, duration, int(sample_rate * duration))
      tone = 0.3 * np.sin(2 * np.pi * frequency * t)
      return sample_rate, tone.astype(np.float32)
  ```

- **Added**: Proper error handling
  ```python
  # NEW:
  if not MOYOYO_AVAILABLE or self.tts is None:
      self.log("ERROR", "MoYoYo TTS not available - cannot synthesize")
      raise RuntimeError("TTS engine not available. Check model paths and configuration.")
  ```

### 3. Bundled MoYoYo TTS Module
**Action**: Copied entire moyoyo_tts module from VoiceDialogue to primespeech node

- **Source**: `/Users/yuechen/home/VoiceDialogue/third_party/moyoyo_tts/`
- **Destination**: `/Users/yuechen/home/conversation/dora/node-hub/dora-primespeech/dora_primespeech/moyoyo_tts/`
- **Import Update**:
  ```python
  # Add local moyoyo_tts to path
  local_moyoyo_path = Path(__file__).parent
  if local_moyoyo_path.exists() and str(local_moyoyo_path) not in sys.path:
      sys.path.insert(0, str(local_moyoyo_path))
  ```

### 4. Unified Logging with send_log
**Files**: `dora_primespeech/moyoyo_tts_wrapper_streaming_fix.py` and `dora_primespeech/main.py`

- **Added**: logger_func parameter to MoYoYoTTSWrapper
  ```python
  def __init__(self, ..., logger_func=None):
      self.logger_func = logger_func
  
  def log(self, level, message):
      """Log a message using the provided logger function or print."""
      if self.logger_func:
          self.logger_func(level, message)
      else:
          print(f"[{level}] {message}")
  ```

- **Updated**: All logger calls to use self.log()
  ```python
  # BEFORE:
  logger.error("MoYoYo TTS not available")
  
  # AFTER:
  self.log("ERROR", "MoYoYo TTS not available")
  ```

- **Updated main.py**: Pass send_log to TTS wrapper
  ```python
  tts_engine = MoYoYoTTSWrapper(
      voice=moyoyo_voice, 
      device=device,
      enable_streaming=enable_streaming,
      chunk_duration=0.3,
      voice_config=voice_config,
      logger_func=lambda level, msg: send_log(node, level, msg)
  )
  ```

### 5. Simplified Model Path Logic
**File**: `dora_primespeech/main.py`

- **Removed**: Complex conditional logic for HuggingFace vs local models
  ```python
  # REMOVED:
  if config.USE_HUGGINGFACE_MODELS:
      # Download from HuggingFace...
  else:
      # Use local VoiceDialogue models...
  ```

- **Simplified**: Always use PRIMESPEECH_MODEL_DIR
  ```python
  # NEW:
  # Always use PRIMESPEECH_MODEL_DIR
  send_log(node, "INFO", "Using PRIMESPEECH_MODEL_DIR for models...")
  
  # Initialize TTS wrapper using PRIMESPEECH_MODEL_DIR
  tts_engine = MoYoYoTTSWrapper(
      voice=moyoyo_voice, 
      device=device,
      enable_streaming=enable_streaming,
      chunk_duration=0.3,
      voice_config=voice_config,
      logger_func=lambda level, msg: send_log(node, level, msg)
  )
  ```

### 6. Removed Unused Configuration
**File**: `dora_primespeech/config.py`

- **Removed**: USE_HUGGINGFACE_MODELS flag
  ```python
  # REMOVED:
  USE_HUGGINGFACE_MODELS = os.getenv("USE_HUGGINGFACE_MODELS", "false").lower() == "true"
  ```

## Environment Variable Usage
The node now uses a single, consistent approach:

```yaml
env:
  PRIMESPEECH_MODEL_DIR: ~/.dora/models/primespeech
```

Models should be organized as:
```
PRIMESPEECH_MODEL_DIR/
└── moyoyo/
    ├── GPT_weights/
    │   └── doubao_best_gpt.ckpt
    ├── SoVITS_weights/
    │   └── doubao_best_sovits.pth
    ├── ref_audios/
    │   └── doubao_ref.wav
    ├── chinese-hubert-base/
    └── chinese-roberta-wwm-ext-large/
```

## Testing Commands

### Test import availability:
```bash
python -c "from dora_primespeech.moyoyo_tts_wrapper_streaming_fix import MOYOYO_AVAILABLE; print(f'MOYOYO_AVAILABLE: {MOYOYO_AVAILABLE}')"
```

### Test with simple pipeline:
```bash
cd /Users/yuechen/home/conversation/dora/examples/voice-chatbot
dora start test_segmenter.yml --attach
```

### Full voice chat pipeline:
```bash
cd /Users/yuechen/home/conversation/dora/examples/voice-chatbot
dora start voice-chat-no-aec.yml --attach
```

## Result
The PrimeSpeech node is now:
- ✅ **Standalone** - No external VoiceDialogue dependencies
- ✅ **Portable** - Uses environment variables for configuration
- ✅ **Self-contained** - Includes all necessary TTS code
- ✅ **Properly failing** - Shows clear errors instead of playing tones
- ✅ **Consistent logging** - Uses send_log throughout

## Dependencies
All required dependencies are listed in `pyproject.toml`:
- Core: torch, torchaudio, transformers, librosa
- Audio: soundfile, ffmpeg-python
- Chinese support: pypinyin, jieba, cn2an
- Utilities: pyyaml, tqdm, scipy