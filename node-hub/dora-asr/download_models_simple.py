#!/usr/bin/env python3
"""
Simple model downloader for ASR models.
Downloads pre-packaged model archives from a CDN or direct links.
"""

import os
import sys
from pathlib import Path
import urllib.request
import tarfile
import zipfile
import shutil

# Model download URLs (these are example URLs - replace with actual ones)
MODEL_URLS = {
    "funasr": {
        "description": "FunASR Chinese models (Paraformer + Punctuation)",
        "size": "~500MB",
        # Replace with actual URL to model archive
        "url": "https://example.com/funasr-models.tar.gz",
        "local_file": "funasr-models.tar.gz",
        "extract_to": "funasr"
    },
    "whisper": {
        "description": "OpenAI Whisper models",
        "size": "Model downloaded automatically on first use",
        "url": None,  # Whisper downloads automatically
        "info": "Whisper models download automatically when first used"
    }
}

def download_file(url, dest_path, description=""):
    """Download file with progress indicator."""
    def download_progress(block_num, block_size, total_size):
        downloaded = block_num * block_size
        percent = min(downloaded * 100 / total_size, 100)
        bar_length = 40
        filled = int(bar_length * percent / 100)
        bar = '=' * filled + '-' * (bar_length - filled)
        sys.stdout.write(f'\r  [{bar}] {percent:.1f}%')
        sys.stdout.flush()
    
    print(f"Downloading {description}...")
    print(f"  URL: {url}")
    print(f"  Destination: {dest_path}")
    
    try:
        urllib.request.urlretrieve(url, dest_path, download_progress)
        print()  # New line after progress bar
        return True
    except Exception as e:
        print(f"\n✗ Download failed: {e}")
        return False

def extract_archive(archive_path, extract_to):
    """Extract tar.gz or zip archive."""
    print(f"Extracting {archive_path.name}...")
    
    try:
        if archive_path.suffix == '.gz':
            with tarfile.open(archive_path, 'r:gz') as tar:
                tar.extractall(extract_to)
        elif archive_path.suffix == '.zip':
            with zipfile.ZipFile(archive_path, 'r') as zip_ref:
                zip_ref.extractall(extract_to)
        else:
            print(f"✗ Unknown archive format: {archive_path.suffix}")
            return False
        
        print(f"✓ Extracted to {extract_to}")
        return True
        
    except Exception as e:
        print(f"✗ Extraction failed: {e}")
        return False

def setup_funasr_models(models_dir):
    """Setup FunASR models directory structure."""
    funasr_dir = models_dir / "funasr"
    funasr_dir.mkdir(parents=True, exist_ok=True)
    
    # Expected model directories
    asr_model = "speech_seaco_paraformer_large_asr_nat-zh-cn-16k-common-vocab8404-pytorch"
    punc_model = "punc_ct-transformer_cn-en-common-vocab471067-large"
    
    asr_path = funasr_dir / asr_model
    punc_path = funasr_dir / punc_model
    
    if asr_path.exists() and punc_path.exists():
        print("✓ FunASR models already installed")
        return True
    
    print("\nFunASR Model Setup")
    print("-" * 40)
    print("FunASR models need to be downloaded from ModelScope.")
    print("\nOption 1: Use the ModelScope Git method")
    print("  python download_funasr_models.py")
    print("\nOption 2: Manual download")
    print("  1. Visit https://modelscope.cn/models/damo/speech_seaco_paraformer_large")
    print("  2. Download the model files")
    print(f"  3. Extract to: {funasr_dir}")
    print("\nOption 3: Use pre-downloaded archive")
    print("  If you have a funasr-models.tar.gz archive:")
    print(f"  1. Place it in: {models_dir}")
    print("  2. Run this script again")
    
    # Check for local archive
    archive_path = models_dir / "funasr-models.tar.gz"
    if archive_path.exists():
        print(f"\n✓ Found archive: {archive_path}")
        if extract_archive(archive_path, funasr_dir):
            print("✓ FunASR models extracted successfully")
            return True
    
    return False

def main():
    """Main function."""
    # Get models directory
    models_dir = os.getenv("ASR_MODELS_DIR")
    if not models_dir:
        models_dir = Path.home() / ".dora" / "models" / "asr"
    else:
        models_dir = Path(models_dir)
    
    models_dir.mkdir(parents=True, exist_ok=True)
    
    print("=" * 60)
    print("ASR Model Setup")
    print("=" * 60)
    print(f"Models directory: {models_dir}\n")
    
    # Check what's needed
    print("Checking installed models...")
    
    # Whisper
    print("\n1. Whisper Models")
    print("-" * 40)
    print("✓ Whisper models download automatically on first use")
    print("  No manual download required")
    
    # FunASR
    print("\n2. FunASR Models (Chinese)")
    print("-" * 40)
    funasr_ready = setup_funasr_models(models_dir)
    
    # Summary
    print("\n" + "=" * 60)
    print("Setup Summary")
    print("=" * 60)
    
    print("\n✓ Whisper: Ready (auto-download)")
    if funasr_ready:
        print("✓ FunASR: Ready")
    else:
        print("⚠ FunASR: Not installed (see instructions above)")
    
    print(f"\nModels location: {models_dir}")
    print("\nUsage in dora configuration:")
    print("  env:")
    print("    ASR_ENGINE: auto  # or whisper/funasr")
    print(f"    ASR_MODELS_DIR: {models_dir}")
    
    if not funasr_ready:
        print("\n⚠ Note: ASR will work with Whisper only until FunASR models are installed")

if __name__ == "__main__":
    main()