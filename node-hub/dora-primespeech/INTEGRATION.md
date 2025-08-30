# Integrating Real GPT-SoVITS Models

## ⚠️ Important Notice

The current implementation uses **placeholder audio generation** (test tones) instead of actual speech synthesis. This is why you hear noise/tones instead of voice.

To get real voice synthesis working, you need to integrate the actual GPT-SoVITS models.

## Option 1: Use GPT-SoVITS Official Implementation

### 1. Install GPT-SoVITS

```bash
# Clone the official repository
git clone https://github.com/RVC-Boss/GPT-SoVITS.git
cd GPT-SoVITS

# Install requirements
pip install -r requirements.txt
```

### 2. Download Pre-trained Models

```bash
# Download base models
python download_models.py
```

### 3. Update the Engine

Replace the `SimplifiedGPTSoVITS` class in `gpt_sovits.py` with:

```python
import sys
sys.path.append('/path/to/GPT-SoVITS')

from GPT_SoVITS.inference_webui import change_gpt_weights, change_sovits_weights
from GPT_SoVITS.inference_webui import get_tts_wav

class GPTSoVITSEngine:
    def __init__(self, model_paths, voice_config):
        self.model_paths = model_paths
        self.voice_config = voice_config
        
        # Load models
        change_gpt_weights(str(model_paths["gpt_model"]))
        change_sovits_weights(str(model_paths["sovits_model"]))
        
        # Load reference audio
        self.ref_audio_path = str(model_paths["reference_audio"])
        self.ref_text = voice_config.get("prompt_text", "")
        
    def synthesize(self, text):
        # Use GPT-SoVITS API
        audio_list = get_tts_wav(
            ref_wav_path=self.ref_audio_path,
            prompt_text=self.ref_text,
            prompt_language=self.voice_config.get("prompt_lang", "zh"),
            text=text,
            text_language=self.voice_config.get("text_lang", "zh"),
            how_to_cut="不切",  # Don't cut
            top_k=self.voice_config.get("top_k", 5),
            top_p=self.voice_config.get("top_p", 1),
            temperature=self.voice_config.get("temperature", 1),
            speed=self.voice_config.get("speed_factor", 1.0)
        )
        
        # Combine audio chunks
        sample_rate = 32000  # GPT-SoVITS default
        audio_data = np.concatenate([np.frombuffer(chunk, dtype=np.int16) for chunk in audio_list])
        audio_data = audio_data.astype(np.float32) / 32768.0
        
        return audio_data, sample_rate
```

## Option 2: Use MoYoYo's Simplified Version

### 1. Install MoYoYo TTS

```bash
# Clone MoYoYo's implementation
git clone https://github.com/MoYoYoTech/MoYoYo-TTS.git
cd MoYoYo-TTS

# Install requirements
pip install -r requirements.txt
```

### 2. Update Integration

```python
import sys
sys.path.append('/path/to/MoYoYo-TTS')

from moyoyo_tts import TTSModule, TTS_Config

class MoYoYoEngine:
    def __init__(self, model_paths, voice_config):
        # Create config
        config_dict = {
            "gpt_path": str(model_paths["gpt_model"]),
            "sovits_path": str(model_paths["sovits_model"]),
            "ref_audio_path": str(model_paths["reference_audio"]),
            "prompt_text": voice_config.get("prompt_text", ""),
            "prompt_lang": voice_config.get("prompt_lang", "zh"),
            "text_lang": voice_config.get("text_lang", "zh"),
            # ... other parameters
        }
        
        tts_config = TTS_Config(config_dict)
        self.tts_module = TTSModule(tts_config)
        self.tts_module.setup_inference_params(
            ref_audio=str(model_paths["reference_audio"]),
            **voice_config
        )
    
    def synthesize(self, text):
        (sample_rate, audio_data), *_ = self.tts_module.generate_audio(text)
        return audio_data, sample_rate
```

## Option 3: Use ONNX Optimized Version

For faster inference without PyTorch:

### 1. Convert Models to ONNX

```bash
# Use the export script
python export_onnx.py \
    --gpt-path models/GPT_weights/doubao_best_gpt.ckpt \
    --sovits-path models/SoVITS_weights/doubao_best_sovits.pth \
    --output-dir models/onnx/
```

### 2. Use ONNX Runtime

```python
import onnxruntime as ort

class ONNXEngine:
    def __init__(self, model_paths, voice_config):
        # Load ONNX models
        self.gpt_session = ort.InferenceSession(
            str(model_paths["gpt_model"].replace(".ckpt", ".onnx"))
        )
        self.sovits_session = ort.InferenceSession(
            str(model_paths["sovits_model"].replace(".pth", ".onnx"))
        )
        
    def synthesize(self, text):
        # ONNX inference implementation
        # ... (implement based on model architecture)
        pass
```

## Required Dependencies

Add these to `pyproject.toml`:

```toml
dependencies = [
    # Existing dependencies...
    
    # For GPT-SoVITS
    "librosa>=0.10.0",
    "soundfile>=0.12.0",
    "pypinyin>=0.50.0",
    "jieba>=0.42.1",
    "cn2an>=0.5.22",
    "transformers>=4.35.0",
    "g2pw",  # Grapheme to phoneme for Chinese
    
    # Optional for ONNX
    "onnxruntime>=1.16.0",
]
```

## Testing Real Voice

After integration:

```bash
# Test with real voice
cd examples/primespeech-demo
dora start tts_complete.yml --name test-real-voice

# You should hear actual speech, not tones!
```

## Troubleshooting

### Still hearing noise?
1. Check model files are downloaded correctly
2. Verify model paths in configuration
3. Check console for error messages
4. Ensure all dependencies are installed

### Models not loading?
1. Check file paths are correct
2. Verify model format matches implementation
3. Check GPU/CPU compatibility
4. Try with smaller models first

### Poor voice quality?
1. Ensure reference audio is high quality
2. Adjust temperature and top_k parameters
3. Use appropriate prompt text for the voice
4. Check sample rate matches your audio system

## Resources

- GPT-SoVITS Official: https://github.com/RVC-Boss/GPT-SoVITS
- Model Downloads: https://huggingface.co/MoYoYoTech/tone-models
- Documentation: https://github.com/RVC-Boss/GPT-SoVITS/wiki

## Note

The placeholder implementation is intentional to:
1. Allow testing the Dora node structure without large model downloads
2. Demonstrate the interface and data flow
3. Provide a working example that can be enhanced with real models

Once you integrate the real models following the instructions above, you'll get high-quality voice synthesis instead of test tones.