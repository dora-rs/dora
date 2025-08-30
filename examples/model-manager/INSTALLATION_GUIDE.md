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

# For Whisper support (optional, choose one):
# Option A: C++ Whisper (faster, might have build issues)
pip install -e "node-hub/dora-asr[whisper]"
# Option B: OpenAI Whisper (more stable)
pip install -e "node-hub/dora-asr[whisper-openai]"

# Install PrimeSpeech TTS node
pip install -e node-hub/dora-primespeech

# Install other required nodes
pip install -e node-hub/dora-qwen3              # LLM
pip install -e node-hub/dora-text-segmenter     # Text segmentation
pip install -e node-hub/dora-conversation-controller  # Conversation management
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

## Step 3: Set Environment Variables (Optional)

```bash
# Set models directory (optional, defaults to ~/.dora/models)
export PRIMESPEECH_MODEL_DIR=~/.dora/models/primespeech
export ASR_MODELS_DIR=~/.dora/models/asr
```

## Step 4: Run Voice Chat

```bash
cd examples/mac-aec-chat  # or your example directory
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