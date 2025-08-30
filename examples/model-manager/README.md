# Model Manager

Universal model downloader and manager for HuggingFace models, PrimeSpeech TTS voices, and FunASR models.

## Features

- Download ANY model from HuggingFace Hub
- Download FunASR models from ModelScope for Chinese ASR
- List all downloaded models across different locations
- Remove downloaded models with confirmation
- Manage PrimeSpeech TTS voice models
- Support for custom download directories
- File pattern filtering for selective downloads
- Resume capability for interrupted downloads

## Installation

The script automatically installs required dependencies:
- `huggingface-hub`
- `tqdm` (for progress bars)

## Usage

### List Downloaded Models

List all models in your HuggingFace cache and local directories:

```bash
./download_models.py --list
```

This scans:
- `~/.cache/huggingface/hub/` - HuggingFace cache
- `~/.dora/models/` - Dora models directory
- Shows model sizes and file counts

### Remove Models

Remove downloaded models to free up space:

```bash
# Remove a HuggingFace model (with confirmation)
./download_models.py --remove mlx-community/gemma-3-12b-it-4bit

# Remove FunASR models
./download_models.py --remove funasr

# Remove a specific PrimeSpeech voice
./download_models.py --remove "Luo Xiang"

# Remove all PrimeSpeech voices
./download_models.py --remove all-voices

# Remove PrimeSpeech base models
./download_models.py --remove primespeech-base
```

All removals require confirmation to prevent accidental deletion.

### Download HuggingFace Models

Download any model from HuggingFace Hub:

```bash
# Download a model
./download_models.py --download mlx-community/gemma-3-12b-it-4bit

# Download to custom directory
./download_models.py --download mlx-community/gemma-3-12b-it-4bit --hf-dir ~/my-models/gemma

# Download only specific file types
./download_models.py --download mlx-community/gemma-3-12b-it-4bit --patterns "*.safetensors" "*.json"

# Download specific revision/branch
./download_models.py --download mlx-community/gemma-3-12b-it-4bit --revision main
```

### FunASR Models

Download FunASR models for Chinese ASR:

```bash
# Download FunASR models (Paraformer + Punctuation)
./download_models.py --download funasr

# Remove FunASR models
./download_models.py --remove funasr
```

FunASR models are stored in `~/.dora/models/asr/funasr/` by default.

### PrimeSpeech TTS Models

List available voices:

```bash
./download_models.py --list-voices
```

Download PrimeSpeech models:

```bash
# Download base models (Chinese Hubert & Roberta)
./download_models.py --download primespeech-base

# Download specific voice
./download_models.py --voice Doubao
# Or using --download
./download_models.py --download "Luo Xiang"

# Download all voices
./download_models.py --voice all

# Custom models directory
./download_models.py --voice Doubao --models-dir ~/my-tts-models
```

Remove PrimeSpeech models:

```bash
# Remove specific voice
./download_models.py --remove "Luo Xiang"

# Remove all voices
./download_models.py --remove all-voices

# Remove base models
./download_models.py --remove primespeech-base
```

## Available PrimeSpeech Voices

### Chinese Voices
- **Doubao** - General purpose voice
- **Luo Xiang** - Legal expert style
- **Yang Mi** - Female celebrity voice
- **Zhou Jielun** - Jay Chou style
- **Ma Yun** - Jack Ma style
- **BYS** - Youth voice
- **Ma Baoguo** - Elder martial arts style
- **Shen Yi** - Professional analyst

### English Voices
- **Maple** - Female English voice
- **Cove** - Male English voice
- **Ellen** - Talk show host style
- **Juniper** - Narrative voice
- **Trump** - Presidential style

## Model Storage Locations

- **HuggingFace models**: `~/.cache/huggingface/hub/`
- **PrimeSpeech models**: `~/.dora/models/primespeech/`
- **FunASR models**: `~/.dora/models/asr/funasr/`
- **Other Dora models**: `~/.dora/models/`

## Examples

```bash
# List all local models
./download_models.py --list

# Download Qwen model
./download_models.py --download Qwen/Qwen3-8B-MLX-4bit

# Download Whisper model
./download_models.py --download openai/whisper-large-v3

# Download Llama model to custom directory
./download_models.py --download meta-llama/Llama-2-7b-hf --hf-dir ~/llama-models

# Download only model weights (skip tokenizer files)
./download_models.py --download mlx-community/gemma-3-12b-it-4bit --patterns "*.safetensors" "*.bin"

# Download FunASR models for Chinese ASR
./download_models.py --download funasr

# Download PrimeSpeech voice
./download_models.py --voice "Luo Xiang"

# Remove a HuggingFace model
./download_models.py --remove mlx-community/gemma-3-12b-it-4bit

# Remove FunASR models
./download_models.py --remove funasr

# Remove a specific voice
./download_models.py --remove "Luo Xiang"
```

## Troubleshooting

### Script Not Found
Make sure to run with `./` prefix or full path:
```bash
./download_models.py --list
# OR
python3 ./download_models.py --list
```

### PrimeSpeech Not Available
If you see "PrimeSpeech not available", the PrimeSpeech modules are not installed. You can still download any HuggingFace model.

### Large Model Downloads
For large models (>5GB), downloads may take time. The script supports resume, so you can restart if interrupted.

### Permission Errors
If you get permission errors, try using a different download directory:
```bash
./download_models.py --download model-name --hf-dir ~/my-models
```

## Notes

- Downloads are automatically resumed if interrupted
- Model files are cached to avoid re-downloading
- Use `--patterns` to save bandwidth by downloading only needed files
- The script detects HuggingFace repos by checking for "/" in the model name