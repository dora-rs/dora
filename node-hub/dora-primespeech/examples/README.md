# PrimeSpeech Examples

## Model Manager Example

The `model_manager_example.py` script demonstrates how to use the ModelManager class to download and manage GPT-SoVITS models from HuggingFace.

### Usage

```bash
# Run the model manager example
python examples/model_manager_example.py
```

This example shows:
- Listing available voices from configuration
- Checking which models are already downloaded
- Downloading specific voice models
- Getting model paths and information
- Managing model storage

## Model Download Script

The `download_models.py` script provides a command-line interface for downloading models.

### Usage

```bash
# Download all models
python examples/download_models.py --voice all

# Download specific voice
python examples/download_models.py --voice "Luo Xiang"

# List available voices
python examples/download_models.py --list

# Specify custom models directory
python examples/download_models.py --voice all --models-dir /path/to/models
```

### Supported Voices

- **Chinese**: Doubao, Luo Xiang, Yang Mi, Zhou Jielun, Ma Yun, BYS, Ma Baoguo, Shen Yi
- **English**: Maple, Cove, Ellen, Juniper, Trump

### Default Model Location

Models are downloaded to `~/.dora/models/primespeech/moyoyo/` by default.