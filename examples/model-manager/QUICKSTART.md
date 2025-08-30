# 🚀 Model Manager - Quick Start Guide

A standalone tool for managing AI models used in Dora pipelines.

## Installation

### Option 1: Basic (HuggingFace only)
```bash
cd /Users/yuechen/home/conversation/dora/examples/model-manager
pip install -r requirements.txt
```

### Option 2: Full (HuggingFace + ModelScope)
```bash
cd /Users/yuechen/home/conversation/dora/examples/model-manager
pip install -r requirements-full.txt
```

### Option 3: Interactive Setup
```bash
cd /Users/yuechen/home/conversation/dora/examples/model-manager
./setup.sh
```

## Basic Usage

### List all cached models
```bash
python download_models.py --list
```

### Download a model
```bash
# Using shortcuts
python download_models.py --download qwen3-8b-mlx-4bit
python download_models.py --download whisper-base

# Using repo ID
python download_models.py --repo Qwen/Qwen3-32B-MLX-6bit
```

### Remove a model
```bash
python download_models.py --remove whisper-tiny --force
```

### Clean incomplete downloads
```bash
python download_models.py --clean
```

## Common Model Sets for Dora

### For Voice Assistant
```bash
python download_models.py --download whisper-base      # ASR
python download_models.py --download qwen3-8b-mlx-4bit  # LLM
```

### For Chinese Voice Assistant
```bash
python download_models.py --download funasr-paraformer     # Chinese ASR
python download_models.py --download funasr-punctuation    # Punctuation
python download_models.py --download qwen3-8b-mlx-4bit     # LLM
```

## Files in this directory

- `download_models.py` - Main script
- `requirements.txt` - Basic dependencies
- `requirements-full.txt` - All dependencies including ModelScope
- `setup.sh` - Interactive setup script
- `config.yaml` - Optional configuration
- `README.md` - Full documentation

## Quick Reference

| Command | Description |
|---------|-------------|
| `--list` | Show all cached models |
| `--download NAME` | Download by shortcut name |
| `--repo REPO_ID` | Download by repository ID |
| `--remove MODEL` | Remove a model |
| `--clean` | Clean incomplete downloads |
| `--force` | Skip confirmation prompts |
| `--show-models` | Show available shortcuts |

See `README.md` for complete documentation.