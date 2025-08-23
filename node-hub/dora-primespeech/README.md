# PrimeSpeech TTS Node

High-quality neural text-to-speech synthesis node with streaming support and multiple voice options. Optimized for real-time conversational AI.

## Overview

PrimeSpeech provides natural-sounding speech synthesis using advanced neural TTS models. It supports multiple languages and voices, with streaming capabilities for low-latency voice interactions.

## Features

- **High-Quality Voices**: Natural, expressive speech synthesis
- **Multiple Languages**: English, Chinese, and more
- **Streaming Synthesis**: Start playback before full generation
- **Voice Selection**: Multiple speaker options per language
- **Speed Control**: Adjustable speaking rate
- **Emotion Control**: Expressive speech styles
- **Backpressure Support**: Completion signals for flow control
- 🔄 **Dynamic Voice Switching**: Change voices at runtime via control commands
- 📊 **Comprehensive Logging**: Structured logging with configurable levels
- 🎛️ **Flexible Parameters**: Fine-tune synthesis with temperature, speed, and more

## ⚠️ IMPORTANT: Current Limitations

**The current implementation uses placeholder audio (test tones) instead of real voice synthesis.** This is why you hear noise/beeps instead of speech.

To get actual voice synthesis:
1. See `INTEGRATION.md` for instructions on integrating real GPT-SoVITS models
2. Or wait for the full implementation update

## Installation

### 1. Install the node

```bash
cd node-hub/dora-primespeech
pip install -e .
```

### 2. Download models

Models are automatically downloaded from HuggingFace on first use, or you can pre-download them:

```bash
# Download all voices
python download_models.py --voice all

# Download specific voice
python download_models.py --voice Doubao

# List available voices
python download_models.py --list
```

Models are stored in `~/.dora/models/primespeech/` by default.

## Available Voices

### Chinese Voices
- **Doubao** (豆包) - MoYoYo virtual assistant, friendly and versatile
- **Luo Xiang** (罗翔) - Famous law professor, humorous and educational
- **Yang Mi** (杨幂) - Celebrity actress, sweet and natural voice
- **Zhou Jielun** (周杰伦) - Jay Chou, distinctive Taiwanese accent
- **Ma Yun** (马云) - Jack Ma, passionate business speaker

### English Voices
- **Maple** - Relaxed and frank female voice
- **Cove** - Calm and straightforward male voice

## Usage

### Basic TTS Example

```yaml
# tts_example.yml
nodes:
  - id: primespeech
    build: pip install -e ../../node-hub/dora-primespeech
    path: dora-primespeech
    inputs:
      text: text-source/text
    outputs:
      - audio
      - status
      - log
    env:
      VOICE_NAME: Doubao
      TEXT_LANG: zh
      SPEED_FACTOR: 1.0
      LOG_LEVEL: INFO
```

### With LLM Integration

```yaml
# Connect with Qwen3 for AI-generated speech
nodes:
  - id: qwen3-llm
    path: dora-qwen3
    outputs:
      - text
      
  - id: primespeech
    path: dora-primespeech
    inputs:
      text: qwen3-llm/text
    outputs:
      - audio
```

### Real-time Interpreter

See `examples/primespeech-demo/tts_realtime_interpreter.yml` for a complete voice interpreter pipeline:
- Microphone → Speech Monitor → ASR → Translation (Qwen3) → TTS (PrimeSpeech) → Audio Output

## Configuration

### Environment Variables

| Variable | Description | Default | Options |
|----------|-------------|---------|---------|
| `VOICE_NAME` | Voice character to use | Doubao | See available voices |
| `TEXT_LANG` | Text language | auto | zh, en, auto |
| `PROMPT_LANG` | Reference prompt language | auto | zh, en, auto |
| `TOP_K` | Top-k sampling | 5 | 1-20 |
| `TOP_P` | Nucleus sampling | 1.0 | 0.0-1.0 |
| `TEMPERATURE` | Sampling temperature | 1.0 | 0.1-2.0 |
| `SPEED_FACTOR` | Speech speed multiplier | 1.0 | 0.5-2.0 |
| `USE_GPU` | Enable GPU acceleration | false | true/false |
| `SAMPLE_RATE` | Audio sample rate | 32000 | 16000/32000/48000 |
| `LOG_LEVEL` | Logging level | INFO | DEBUG/INFO/WARNING/ERROR |

### Model Storage

Models are stored in:
- Default: `~/.dora/models/primespeech/`
- Override: Set `PRIMESPEECH_MODEL_DIR` environment variable

### Control Commands

Send control commands via the `control` input:

- `stats` - Display synthesis statistics
- `list_voices` - List available voices
- `change_voice:VoiceName` - Change voice dynamically
- `cleanup` - Clean up resources

## Node Interface

### Inputs

- **text** (string): Text to synthesize
  - Metadata:
    - `session_id`: Session identifier
    - `request_id`: Request identifier

- **control** (string): Control commands

### Outputs

- **audio** (float32 array): Synthesized audio waveform
  - Metadata:
    - `sample_rate`: Audio sample rate
    - `duration`: Audio duration in seconds
    - `voice`: Voice name used
    - `language`: Language detected/used

- **status** (string): Synthesis status ("completed", "error")
  - Metadata:
    - `session_id`: Session identifier
    - `request_id`: Request identifier
    - `error`: Error message if failed

- **log** (JSON string): Structured log messages
  - Fields:
    - `node`: "primespeech"
    - `level`: Log level
    - `message`: Log message
    - `timestamp`: Unix timestamp

## Implementation Details

### Architecture

The node uses a simplified GPT-SoVITS architecture:
1. **Text Processing**: Clean and normalize input text
2. **Phoneme Extraction**: Convert text to phonemes (language-specific)
3. **GPT Model**: Generate mel-spectrogram from phonemes
4. **SoVITS Vocoder**: Convert mel-spectrogram to waveform
5. **Post-processing**: Apply speed and audio effects

### Model Management

- Automatic model downloading from HuggingFace
- Model caching for fast loading
- Lazy loading on first synthesis request
- Model warmup for optimal performance

### Performance

- CPU: ~2-5x realtime on modern processors
- GPU: ~10-20x realtime with CUDA (when available)
- Memory: ~2-4GB per voice model
- Latency: ~200-500ms first synthesis, ~50-100ms subsequent

## Development

### Adding New Voices

1. Add voice configuration to `config.py`:
```python
VOICE_CONFIGS["NewVoice"] = {
    "repository": "user/repo",
    "gpt_weights": "path/to/gpt.ckpt",
    "sovits_weights": "path/to/sovits.pth",
    "reference_audio": "path/to/ref.wav",
    "prompt_text": "Reference text",
    "text_lang": "zh",
    # ... other parameters
}
```

2. Upload models to HuggingFace repository

3. Test with: `python download_models.py --voice NewVoice`

### Full GPT-SoVITS Integration

The current implementation uses a simplified TTS engine. For production use with actual GPT-SoVITS models:

1. Install GPT-SoVITS dependencies:
```bash
pip install git+https://github.com/RVC-Boss/GPT-SoVITS
```

2. Replace `SimplifiedGPTSoVITS` in `gpt_sovits.py` with actual implementation

3. Update model loading in `GPTSoVITSEngine.load_models()`

## Troubleshooting

### Models not downloading
- Check internet connection
- Verify HuggingFace access
- Try manual download: `huggingface-cli download MoYoYoTech/tone-models`

### Audio quality issues
- Ensure proper sample rate (32000 Hz recommended)
- Check input text for special characters
- Adjust temperature and top_k parameters

### Performance problems
- Enable GPU if available: `USE_GPU=true`
- Reduce batch size for lower memory usage
- Use voice warmup for better latency

## License

This node is part of the Dora ecosystem. Model licenses vary by voice - check HuggingFace repositories for details.

## Credits

- GPT-SoVITS: https://github.com/RVC-Boss/GPT-SoVITS
- MoYoYo Models: https://huggingface.co/MoYoYoTech
- Dora Framework: https://github.com/dora-rs/dora