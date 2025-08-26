# Model Manager

Universal model downloader and manager for HuggingFace models and PrimeSpeech TTS voices.

## Features

- Download ANY model from HuggingFace Hub
- List all downloaded models across different locations
- Remove downloaded models with confirmation
- Clean incomplete downloads
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
# Remove a specific model (with confirmation)
./download_models.py --remove mlx-community/gemma-3-12b-it-4bit

# Remove by partial name (matches all containing the pattern)
./download_models.py --remove gemma

# Skip confirmation prompt
./download_models.py --remove mlx-community/gemma-3-12b-it-4bit --force
```

### Clean Incomplete Downloads

Clean up failed or incomplete downloads:

```bash
./download_models.py --clean
```

This removes files with extensions: `.incomplete`, `.downloading`, `.tmp`, `.part`

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

# Download all voices
./download_models.py --voice all

# Custom models directory
./download_models.py --voice Doubao --models-dir ~/my-tts-models
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

# Download PrimeSpeech voice
./download_models.py --voice "Luo Xiang"
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