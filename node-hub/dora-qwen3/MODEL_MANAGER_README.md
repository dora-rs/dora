# 🤖 Model Download Manager for Dora Nodes

A comprehensive model management tool for downloading, listing, and removing AI models from HuggingFace and ModelScope repositories. Supports popular models like Qwen, Whisper, FunASR, and more with progress tracking and resume capability.

## 📋 Table of Contents
- [Features](#features)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [Commands](#commands)
- [Model Sources](#model-sources)
- [Supported Models](#supported-models)
- [Examples](#examples)
- [Cache Locations](#cache-locations)
- [Troubleshooting](#troubleshooting)

## ✨ Features

- **📥 Download models** with progress bars and resume capability
- **📊 List cached models** organized by organization/owner
- **🗑️ Remove models** with safety confirmation
- **🧹 Clean incomplete downloads** to free up space
- **🌍 Dual source support**: HuggingFace and ModelScope
- **⚡ Smart shortcuts** for popular models
- **📈 Progress tracking** with real-time download status
- **💾 Automatic caching** with size reporting
- **🔄 Resume interrupted downloads** automatically

## 📦 Installation

The script is located at:
```bash
/Users/yuechen/home/conversation/dora/node-hub/dora-qwen3/download_models.py
```

### Required Dependencies

The script will auto-install missing dependencies:
- `tqdm` - Progress bars
- `huggingface-hub` - HuggingFace model downloads
- `modelscope` (optional) - ModelScope model downloads

To manually install:
```bash
pip install tqdm huggingface-hub
pip install modelscope  # Optional, for Chinese models
```

## 🚀 Quick Start

### List all cached models
```bash
python download_models.py --list
```

### Download a model
```bash
# Using shortcut
python download_models.py --download qwen3-8b-mlx-4bit

# Using repo ID
python download_models.py --repo Qwen/Qwen3-8B-MLX-4bit
```

### Remove a model
```bash
python download_models.py --remove whisper-tiny --force
```

## 📚 Commands

### `--list`
Lists all locally cached models, organized by organization.

**Output Example:**
```
📚 LOCALLY CACHED MODELS
============================================================

🤗 HuggingFace Models:

  [Qwen]
    • Qwen3-8B-MLX-4bit                             [  4.07GB]
    • Qwen3-14B-MLX-4bit                            [  7.32GB]
    • Qwen3-8B-GGUF                                 [  4.68GB]

  [openai]
    • whisper-tiny                                  [  0.29GB]
    • whisper-base                                  [  0.14GB]

🔬 ModelScope Models:

  [iic]
    • speech_paraformer-large_asr_nat-zh-cn-16k     [  0.93GB]
    • punc_ct-transformer_cn-en-common              [  1.11GB]
```

### `--download <model_name>`
Downloads a model using predefined shortcuts.

**Available Shortcuts:**
| Shortcut | Model | Size |
|----------|-------|------|
| `qwen3-8b-mlx-4bit` | Qwen/Qwen3-8B-MLX-4bit | ~4.5GB |
| `qwen3-8b-mlx-8bit` | Qwen/Qwen3-8B-MLX-8bit | ~8GB |
| `qwen3-14b-mlx-4bit` | Qwen/Qwen3-14B-MLX-4bit | ~7.5GB |
| `qwen3-32b-mlx-6bit` | Qwen/Qwen3-32B-MLX-6bit | ~20GB |
| `whisper-large-v3` | openai/whisper-large-v3 | ~3GB |
| `whisper-medium` | openai/whisper-medium | ~1.5GB |
| `whisper-small` | openai/whisper-small | ~500MB |
| `whisper-base` | openai/whisper-base | ~150MB |
| `whisper-tiny` | openai/whisper-tiny | ~40MB |
| `distil-whisper-large-v3` | distil-whisper/distil-large-v3 | ~1.5GB |
| `funasr-paraformer` | FunASR Paraformer (Chinese ASR) | ~200MB |
| `funasr-punctuation` | FunASR Punctuation | ~230MB |

### `--repo <repo_id>`
Downloads any model by its repository ID.

**Examples:**
```bash
# HuggingFace models
python download_models.py --repo openai/whisper-large-v3
python download_models.py --repo Qwen/Qwen3-32B-MLX-6bit
python download_models.py --repo mlx-community/Llama-3.2-3B-Instruct-4bit

# ModelScope models (auto-detected by prefix)
python download_models.py --repo iic/speech_paraformer-large_asr_nat-zh-cn-16k-common-vocab8404-pytorch
python download_models.py --repo damo/speech_eres2net_sv_zh-cn_16k-common
```

### `--source <huggingface|modelscope>`
Explicitly specify the model source (default: auto-detect).

```bash
python download_models.py --repo some/model --source modelscope
```

### `--remove <model>`
Removes a cached model with confirmation prompt.

**Features:**
- Shows model size before deletion
- Shows storage location
- Asks for confirmation (bypass with `--force`)
- Works with shortcuts or repo IDs

```bash
# With confirmation
python download_models.py --remove whisper-tiny

# Without confirmation
python download_models.py --remove whisper-tiny --force

# Remove by repo ID
python download_models.py --remove openai/whisper-base --force
```

### `--remove-multiple <model1> <model2> ...`
Removes multiple models at once.

```bash
python download_models.py --remove-multiple whisper-tiny whisper-base bert-base-uncased --force
```

**Output:**
```
🗑️  Deleting openai/whisper-tiny...
✅ Successfully removed openai/whisper-tiny (0.29GB freed)
🗑️  Deleting openai/whisper-base...
✅ Successfully removed openai/whisper-base (0.14GB freed)

📊 Summary: Removed 2 models, freed 0.43GB
```

### `--clean`
Removes incomplete downloads (partial files from interrupted downloads).

**What it cleans:**
- HuggingFace: `*.incomplete` files
- ModelScope: `*.downloading` files

```bash
python download_models.py --clean
```

**Output:**
```
🧹 Cleaning incomplete downloads...
   Removing: 07cadb9f25677c8d50df603e66a98fbd842cce45047139baeb16e6219a1e807b.incomplete
   Removing: model.pt.downloading
✅ Removed 2 incomplete files
```

### `--show-models`
Shows all available model shortcuts.

```bash
python download_models.py --show-models
```

### `--force`
Skip confirmation prompts when removing models.

```bash
python download_models.py --remove qwen3-8b-mlx-4bit --force
```

## 🌍 Model Sources

### HuggingFace 🤗
- **URL**: https://huggingface.co
- **Cache**: `~/.cache/huggingface/hub/`
- **Models**: International models, Western-focused
- **Progress**: Full progress bars with speed and ETA

### ModelScope 🔬
- **URL**: https://modelscope.cn
- **Cache**: `~/.cache/modelscope/hub/`
- **Models**: Chinese models, Asia-focused (Alibaba)
- **Progress**: Real-time size monitoring
- **Auto-detected prefixes**: `iic/`, `damo/`, `AI-ModelScope/`, `ZhipuAI/`

## 📦 Supported Models

### Qwen Models (MLX for Apple Silicon)
- **Qwen3-8B**: 4-bit (~4.5GB), 8-bit (~8GB)
- **Qwen3-14B**: 4-bit (~7.5GB)
- **Qwen3-32B**: 6-bit (~20GB)
- **GGUF Format**: For CPU/GPU inference

### Whisper Models (Speech Recognition)
- **whisper-large-v3**: ~3GB (best accuracy)
- **whisper-medium**: ~1.5GB
- **whisper-small**: ~500MB
- **whisper-base**: ~150MB
- **whisper-tiny**: ~40MB (fastest)
- **distil-whisper**: Optimized versions

### FunASR Models (Chinese ASR)
- **Paraformer**: Chinese speech recognition
- **Punctuation**: Punctuation restoration
- **VAD**: Voice activity detection

### PrimeSpeech Models (TTS)
- Base model with multiple voice options

## 💡 Examples

### Example 1: Download Qwen3-32B for your Dora pipeline
```bash
# Download the model
python download_models.py --repo Qwen/Qwen3-32B-MLX-6bit

# Verify it's downloaded
python download_models.py --list | grep Qwen3-32B

# Update your YAML config
# MLX_MODEL: "Qwen/Qwen3-32B-MLX-6bit"
```

### Example 2: Clean up old models to free space
```bash
# List all models with sizes
python download_models.py --list

# Remove unused models
python download_models.py --remove-multiple bert-base-uncased bert-base-multilingual-uncased --force

# Clean incomplete downloads
python download_models.py --clean
```

### Example 3: Download Chinese ASR models
```bash
# Download FunASR for Chinese speech recognition
python download_models.py --download funasr-paraformer
python download_models.py --download funasr-punctuation
```

### Example 4: Setup Whisper for multiple languages
```bash
# Download different Whisper sizes
python download_models.py --download whisper-large-v3  # Best accuracy
python download_models.py --download whisper-base      # Balanced
python download_models.py --download whisper-tiny      # Fastest
```

## 📁 Cache Locations

Models are cached locally for fast loading:

| Platform | Location |
|----------|----------|
| HuggingFace | `~/.cache/huggingface/hub/models--{owner}--{model}` |
| ModelScope | `~/.cache/modelscope/hub/models/{owner}/{model}` |

**Check disk usage:**
```bash
du -sh ~/.cache/huggingface/hub/
du -sh ~/.cache/modelscope/hub/
```

## 🔧 Troubleshooting

### Issue: "ModelScope not installed"
**Solution:**
```bash
pip install modelscope
```

### Issue: Download interrupted
**Solution:** Simply run the same command again - downloads auto-resume:
```bash
python download_models.py --repo Qwen/Qwen3-14B-MLX-4bit
```

### Issue: "Model not found"
**Solutions:**
1. Check the repo ID is correct
2. Try with explicit source:
```bash
python download_models.py --repo model/name --source modelscope
```

### Issue: Slow downloads
**Solutions:**
1. ModelScope models may be slow from outside China
2. HuggingFace models may require VPN in some regions
3. Use smaller model variants when possible

### Issue: Out of disk space
**Solutions:**
1. Clean incomplete downloads:
```bash
python download_models.py --clean
```
2. Remove unused models:
```bash
python download_models.py --list  # Check what you have
python download_models.py --remove-multiple model1 model2 --force
```

## 🔄 Progress Display

### HuggingFace Progress
```
📥 Downloading: openai/whisper-tiny
⏳ Downloading from HuggingFace Hub...
Fetching 16 files: 31%|███▏      | 5/16 [00:05<00:11, 1.04s/it]
✅ Downloaded successfully!
   Total size: 0.57GB
```

### ModelScope Progress
```
📥 Downloading: iic/speech_paraformer-large_asr_nat-zh-cn-16k
⏳ Downloading from ModelScope...
📊 Note: ModelScope downloads may not show detailed progress
   Downloaded: 512.7MB  (real-time updates)
✅ Downloaded successfully! Total size: 0.93GB
```

## 🔐 Authentication (Optional)

### HuggingFace Token
For private models or faster downloads:
```bash
export HF_TOKEN=your_token_here
huggingface-cli login
```

### ModelScope Token
For private models:
```bash
export MODELSCOPE_TOKEN=your_token_here
```

## 📝 Integration with Dora Pipelines

After downloading models, update your Dora YAML configuration:

```yaml
# For Qwen LLM
- id: qwen3-llm
  env:
    MLX_MODEL: "Qwen/Qwen3-32B-MLX-6bit"  # Model you downloaded
    MAX_TOKENS: 256

# For Whisper ASR
- id: whisper-asr
  env:
    WHISPER_MODEL: "large-v3"  # Use the model you downloaded

# For FunASR
- id: funasr
  env:
    ASR_MODEL: "paraformer"  # Use downloaded FunASR model
```

## 📊 Storage Management Tips

1. **Regular Cleanup**: Run `--clean` weekly to remove incomplete downloads
2. **Model Rotation**: Keep only actively used models
3. **Size-Performance Trade-off**: Smaller quantized models often work well
4. **Shared Blobs**: HuggingFace shares common files between model versions
5. **Symlinks**: HuggingFace uses symlinks to save space

## 🆘 Getting Help

```bash
# Show help message
python download_models.py --help

# List available shortcuts
python download_models.py --show-models

# Check what's cached
python download_models.py --list
```

## 📄 License

This tool is part of the Dora ecosystem and follows the same licensing terms.

---

**Pro Tips:**
- 🚀 Download models before running pipelines for faster startup
- 💾 Keep frequently used models cached
- 🧹 Clean incomplete downloads after network interruptions
- 📊 Monitor disk usage with `--list` command
- ⚡ Use smaller quantized models for faster inference
- 🌍 Use ModelScope for Chinese models, HuggingFace for others