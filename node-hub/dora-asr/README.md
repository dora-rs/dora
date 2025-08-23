# ASR (Automatic Speech Recognition) Node

Multi-engine speech recognition node with automatic language detection. Supports Whisper and FunASR backends for optimal accuracy and performance.

## Overview

The ASR node provides robust speech-to-text capabilities with automatic backend selection. It intelligently chooses between Whisper (OpenAI) and FunASR (Alibaba) based on availability and language requirements, ensuring reliable transcription across different environments.

## Features

- **Multi-Engine Support**: Automatic fallback between Whisper and FunASR
- **Language Detection**: Automatic detection of Chinese and English
- **High Accuracy**: State-of-the-art transcription models
- **Punctuation Restoration**: Automatic punctuation for better readability
- **Confidence Scores**: Transcription confidence metrics
- **Optimized Models**: Multiple model sizes for speed/accuracy tradeoff
- **GPU Acceleration**: CUDA and Metal support for faster processing

## Installation

```bash
pip install -e dora-asr

# For Whisper backend
pip install openai-whisper

# For FunASR backend (Chinese optimized)
pip install funasr

# For GPU acceleration
pip install torch torchvision torchaudio
```

## Inputs

| Input | Type | Description |
|-------|------|-------------|
| `audio` | Audio array | PCM audio segment (16kHz mono) |
| `control` | String | Control commands: "reset", "status" |

## Outputs

| Output | Type | Description |
|--------|------|-------------|
| `transcription` | String | Transcribed text |
| `language_detected` | String | Detected language code (zh/en) |
| `processing_time` | Float | Time taken for transcription (seconds) |
| `confidence` | Float | Transcription confidence (0-1) |
| `log` | String | Debug and status messages |

## Configuration

```yaml
env:
  # Engine selection
  ASR_ENGINE: auto          # auto, whisper, or funasr
  
  # Language settings
  LANGUAGE: auto           # auto, zh, en, or specific code
  ENABLE_LANGUAGE_DETECTION: true
  
  # Whisper settings
  WHISPER_MODEL: small     # tiny, base, small, medium, large
  WHISPER_DEVICE: auto     # auto, cpu, cuda, or mps
  WHISPER_COMPUTE_TYPE: auto  # auto, int8, float16, float32
  
  # FunASR settings
  FUNASR_MODEL: paraformer-zh  # Model name
  FUNASR_VAD_MODEL: fsmn-vad   # VAD model
  FUNASR_PUNC_MODEL: ct-punc   # Punctuation model
  
  # Processing options
  ENABLE_PUNCTUATION: true
  ENABLE_CONFIDENCE_SCORE: false
  ENABLE_TIMESTAMPS: false
  
  # Performance
  BATCH_SIZE: 1
  NUM_WORKERS: 1
  
  # Logging
  LOG_LEVEL: INFO         # DEBUG, INFO, WARNING, ERROR
```

## Model Selection

### Whisper Models

| Model | Parameters | Speed | Accuracy | Memory |
|-------|------------|-------|----------|---------|
| `tiny` | 39M | Fastest | Good | 1 GB |
| `base` | 74M | Fast | Better | 1 GB |
| `small` | 244M | **Balanced** | Very Good | 2 GB |
| `medium` | 769M | Moderate | Excellent | 5 GB |
| `large` | 1550M | Slow | Best | 10 GB |

### FunASR Models

| Model | Language | Use Case | Speed |
|-------|----------|----------|-------|
| `paraformer-zh` | Chinese | General | Fast |
| `paraformer-en` | English | General | Fast |
| `paraformer-multi` | Multi | Mixed | Moderate |
| `conformer` | Multi | High accuracy | Slow |

## Usage Example

```yaml
nodes:
  - id: asr
    path: dora-asr
    inputs:
      audio:
        source: speech-monitor/audio_segment
        queue_size: 10
    outputs:
      - transcription
      - language_detected
      - processing_time
      - confidence
      - log
    env:
      ASR_ENGINE: auto
      LANGUAGE: auto
      WHISPER_MODEL: small
      ENABLE_PUNCTUATION: true
      ENABLE_LANGUAGE_DETECTION: true
```

## Language Detection

Automatic language detection workflow:

```
Audio Input → Language Detection → Model Selection → Transcription
     ↓              ↓                    ↓              ↓
  16kHz PCM    zh/en/multi      Whisper/FunASR    "你好世界"
```

### Detection Strategy
1. First 3 seconds analyzed for language
2. Confidence threshold: 0.7
3. Fallback to multilingual model if uncertain

## Performance Optimization

### GPU Acceleration (NVIDIA)
```yaml
WHISPER_DEVICE: cuda
WHISPER_COMPUTE_TYPE: float16
BATCH_SIZE: 4
```

### Apple Silicon Optimization
```yaml
WHISPER_DEVICE: mps
WHISPER_COMPUTE_TYPE: float32
```

### CPU Optimization
```yaml
WHISPER_DEVICE: cpu
WHISPER_COMPUTE_TYPE: int8
NUM_WORKERS: 4
```

## Accuracy Tips

### For Chinese
```yaml
ASR_ENGINE: funasr        # Better for Chinese
FUNASR_MODEL: paraformer-zh
ENABLE_PUNCTUATION: true
```

### For English
```yaml
ASR_ENGINE: whisper       # Better for English
WHISPER_MODEL: small
LANGUAGE: en
```

### For Mixed Languages
```yaml
ASR_ENGINE: auto
LANGUAGE: auto
ENABLE_LANGUAGE_DETECTION: true
WHISPER_MODEL: medium     # Better multilingual support
```

## Audio Requirements

- **Sample Rate**: 16000 Hz (required)
- **Channels**: Mono
- **Format**: Float32 or Int16
- **Duration**: 0.5-30 seconds per segment
- **Quality**: Clean audio, minimal background noise

## Processing Pipeline

```python
# 1. Audio preprocessing
audio = normalize_audio(input)
audio = remove_silence(audio)

# 2. Language detection (if enabled)
language = detect_language(audio[:3_seconds])

# 3. Model selection
model = select_model(engine, language)

# 4. Transcription
text = model.transcribe(audio)

# 5. Post-processing
text = add_punctuation(text)
text = correct_spelling(text)
```

## Punctuation Restoration

When `ENABLE_PUNCTUATION: true`:

```
Input:  "今天天气很好我们去公园吧"
Output: "今天天气很好，我们去公园吧。"

Input:  "hello how are you today"
Output: "Hello, how are you today?"
```

## Confidence Scores

When `ENABLE_CONFIDENCE_SCORE: true`:

```json
{
    "transcription": "Hello world",
    "confidence": 0.95,
    "word_confidences": [
        {"word": "Hello", "confidence": 0.98},
        {"word": "world", "confidence": 0.92}
    ]
}
```

## Error Handling

The node handles various error cases:

1. **Engine Fallback**: If FunASR fails, falls back to Whisper
2. **Silent Audio**: Returns empty transcription
3. **Noisy Audio**: Low confidence score warning
4. **Language Mismatch**: Auto-switches model

## Best Practices

1. **Audio Quality**:
   - Use noise reduction preprocessing
   - Ensure proper microphone gain
   - Avoid clipping and distortion

2. **Model Selection**:
   - Start with `small` model
   - Upgrade to `medium/large` if needed
   - Use FunASR for Chinese-heavy content

3. **Performance**:
   - Enable GPU acceleration when available
   - Use appropriate batch size
   - Consider model quantization

4. **Accuracy**:
   - Specify language when known
   - Enable punctuation for readability
   - Use confidence scores for filtering

## Troubleshooting

| Issue | Solution |
|-------|----------|
| Slow transcription | Use smaller model or enable GPU |
| Poor accuracy | Use larger model or clean audio |
| Wrong language | Set LANGUAGE explicitly |
| Missing punctuation | Enable ENABLE_PUNCTUATION |
| Memory errors | Use smaller model or reduce batch size |
| No GPU detected | Check CUDA/MPS installation |

## Benchmarks

Transcription speed (real-time factor on M2 Mac):

| Model | CPU | MPS | Quality |
|-------|-----|-----|---------|
| tiny | 10x | 50x | 85% |
| base | 7x | 35x | 90% |
| small | 4x | 20x | 93% |
| medium | 2x | 10x | 95% |
| large | 0.5x | 5x | 97% |

(10x = 10 times faster than real-time)

## Advanced Configuration

### Custom Vocabulary
```yaml
CUSTOM_VOCABULARY: ["Dora", "PyArrow", "LLM"]
BOOST_WORDS: true
```

### Streaming Mode
```yaml
ENABLE_STREAMING: true
CHUNK_LENGTH_S: 5
STRIDE_LENGTH_S: 1
```

### Multi-Speaker
```yaml
ENABLE_DIARIZATION: true
MAX_SPEAKERS: 2
```

## Integration with Speech Monitor

The ASR node is designed to work with audio segments from speech-monitor:

```yaml
speech-monitor:
  outputs: audio_segment (0.5-10s chunks)
           ↓
asr:
  inputs: audio (processes each segment)
  outputs: transcription (text for each segment)
           ↓
chat-controller:
  accumulates transcriptions into complete questions
```

## Dependencies

- `openai-whisper`: Whisper models
- `funasr`: FunASR models
- `torch`: Neural network backend
- `numpy`: Audio processing
- `scipy`: Signal processing
- `pyarrow`: Dora communication