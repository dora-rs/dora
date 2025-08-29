# Installation Guide for Voice Chat System

This guide helps you set up the complete voice chat system on a new machine.

## Prerequisites

- Python 3.8+ 
- macOS (for mac-aec) or Linux
- Git

## Step 1: Install Dora Nodes

```bash
# Navigate to dora directory
cd ~/path/to/dora

# Install ASR node (with FunASR for Chinese)
pip install -e node-hub/dora-asr



# Install PrimeSpeech TTS node
pip install -e node-hub/dora-primespeech

# Install other required nodes
pip install -e node-hub/dora-qwen3              # LLM
pip install -e node-hub/dora-text-segmenter     # Text segmentation
```

## Step 2: Download Models

```bash
cd examples/model-manager

# Download all required models
python download_models.py --download funasr        # Chinese ASR models
python download_models.py --download g2pw          # Chinese text-to-phoneme
python download_models.py --download primespeech-base  # Base TTS models
python download_models.py --voice Doubao           # Voice model (or other voices)

# List downloaded models to verify
python download_models.py --list
```
You should see something like this depending on how many LLM models you have downloaded. But the primespeech and asr models must be downloaded and shown the correct size.

📦 Scanning for downloaded models...
============================================================

📁 HuggingFace Cache: /Users/yuechen/.cache/huggingface/hub
------------------------------------------------------------
  📦 Qwen/Qwen2.5-0.5B-Instruct                   0.01 GB
  📦 Qwen/Qwen3-14B-MLX-4bit                      7.32 GB
  📦 Qwen/Qwen3-32B-MLX-6bit                     23.85 GB
  📦 Qwen/Qwen3-8B-MLX-4bit                       4.07 GB
  📦 Qwen/Qwen3-8B-MLX-8bit                       7.88 GB
  📦 mlx-community/GLM-4.5-Air-3bit              43.59 GB
📁 /Users/yuechen/.dora/models
------------------------------------------------------------
  📦 asr/funasr/speech_seaco_paraformer_large_asr_nat-zh-cn-16k-common-vocab8404-pytorch     1.85 GB   (FunASR)
  📦 asr/funasr/punc_ct-transformer_cn-en-common-vocab471067-large     2.17 GB   (FunASR)
  📦 asr/whisper                                  0.82 GB   (3 files)
  📦 llm/mlx-community--gemma-2-9b-it-4bit        4.84 GB   (1 files)
  📦 primespeech/G2PWModel                        0.59 GB   (1 files)
  📦 primespeech/moyoyo/GPT_weights               0.29 GB   (2 files)
  📦 primespeech/moyoyo/SoVITS_weights            0.16 GB   (2 files)
  📦 primespeech/moyoyo/chinese-hubert-base       0.18 GB   (1 files)
  📦 primespeech/moyoyo/chinese-roberta-wwm-ext-large     0.61 GB   (1 files)

## Step 3: Set Environment Variables in YAML


# Set models directory (optional, defaults to ~/.dora/models)
PRIMESPEECH_MODEL_DIR=~/.dora/models/primespeech
ASR_MODELS_DIR=~/.dora/models/asr


## Step 4: Run Voice Chat

```bash
cd examples/mac-aec-chat  # or your example directory
dora stop. #stop existing instance one by one
dora up
dora start voice-chat-with-aec.yml
```

## Troubleshooting

### FunASR Loading Issues

If you see "invalid load key, 'v'" error:
1. Delete corrupted models: `rm -rf ~/.dora/models/asr/funasr`
2. Re-download: `python download_models.py --download funasr`
3. If download fails, copy from working machine or download manually from ModelScope

### PyTorch Compatibility

If you encounter PyTorch 2.6+ issues:
```bash
pip install torch==2.5.1 torchaudio==2.5.1
```

### Missing G2PW Model

If PrimeSpeech complains about missing G2PW:
```bash
python download_models.py --download g2pw
```

### pywhispercpp Build Issues

If pywhispercpp fails to install:
1. Use FunASR instead (set `ASR_ENGINE: funasr` in YAML)
2. Or install OpenAI Whisper: `pip install openai-whisper`

## Model Download URLs (Manual Download)

If automatic download fails, manually download from:

- **G2PW**: https://storage.googleapis.com/esun-ai/g2pW/G2PWModel-v2-onnx.zip
  Extract to: `~/.dora/models/primespeech/G2PWModel/`

- **FunASR**: Clone from ModelScope
  - ASR: https://modelscope.cn/models/damo/speech_seaco_paraformer_large_asr_nat-zh-cn-16k-common-vocab8404-pytorch
  - Punctuation: https://modelscope.cn/models/damo/punc_ct-transformer_cn-en-common-vocab471067-large

## Verifying Installation

Run this test to verify everything is installed:

```python
# Test imports
import dora_asr
import dora_primespeech
import funasr
import onnxruntime
import pytorch_lightning

print("✓ All packages installed successfully!")
```