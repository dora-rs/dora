#!/usr/bin/env python3
"""
Download FunASR models for the ASR node.

This script downloads the required FunASR models from ModelScope.
"""

import os
import sys
from pathlib import Path
import subprocess

def download_from_modelscope(model_id, save_dir):
    """Download model from ModelScope using git."""
    save_path = save_dir / model_id.split("/")[-1]
    
    if save_path.exists():
        print(f"✓ Model already exists: {save_path}")
        return save_path
    
    print(f"Downloading {model_id}...")
    print(f"  Saving to: {save_path}")
    
    try:
        # Use git to clone the model
        cmd = [
            "git", "clone",
            f"https://modelscope.cn/{model_id}.git",
            str(save_path)
        ]
        
        result = subprocess.run(cmd, capture_output=True, text=True)
        
        if result.returncode != 0:
            print(f"Error downloading {model_id}:")
            print(result.stderr)
            
            # Alternative: Try with git lfs
            print("\nTrying alternative download method...")
            cmd_alt = [
                "git", "lfs", "clone",
                f"https://modelscope.cn/{model_id}.git",
                str(save_path)
            ]
            subprocess.run(cmd_alt, check=True)
        
        print(f"✓ Downloaded {model_id}")
        return save_path
        
    except subprocess.CalledProcessError as e:
        print(f"✗ Failed to download {model_id}: {e}")
        return None
    except FileNotFoundError:
        print("✗ Git is not installed. Please install git first.")
        print("  macOS: brew install git git-lfs")
        print("  Linux: sudo apt-get install git git-lfs")
        return None


def download_with_python(model_id, save_dir):
    """Alternative download using Python modelscope library."""
    try:
        from modelscope.hub.snapshot_download import snapshot_download
        
        print(f"Downloading {model_id} using ModelScope SDK...")
        save_path = snapshot_download(
            model_id,
            cache_dir=str(save_dir.parent),
            revision='master'
        )
        
        # Move to expected location
        expected_path = save_dir / model_id.split("/")[-1]
        if not expected_path.exists() and save_path != str(expected_path):
            import shutil
            shutil.move(save_path, str(expected_path))
            
        print(f"✓ Downloaded to {expected_path}")
        return expected_path
        
    except ImportError:
        print("ModelScope SDK not installed.")
        print("Install with: pip install modelscope")
        return None
    except Exception as e:
        print(f"Failed to download with ModelScope SDK: {e}")
        return None


def main():
    """Main function to download FunASR models."""
    
    # Get models directory
    models_dir = os.getenv("ASR_MODELS_DIR")
    if not models_dir:
        models_dir = Path.home() / ".dora" / "models" / "asr"
    else:
        models_dir = Path(models_dir)
    
    funasr_dir = models_dir / "funasr"
    funasr_dir.mkdir(parents=True, exist_ok=True)
    
    print("=" * 60)
    print("FunASR Model Downloader")
    print("=" * 60)
    print(f"Models directory: {funasr_dir}")
    print()
    
    # Models to download
    models = [
        {
            "name": "ASR Model (Paraformer)",
            "id": "damo/speech_seaco_paraformer_large_asr_nat-zh-cn-16k-common-vocab8404-pytorch",
            "local_name": "speech_seaco_paraformer_large_asr_nat-zh-cn-16k-common-vocab8404-pytorch"
        },
        {
            "name": "Punctuation Model",
            "id": "damo/punc_ct-transformer_cn-en-common-vocab471067-large",
            "local_name": "punc_ct-transformer_cn-en-common-vocab471067-large"
        }
    ]
    
    print("Models to download:")
    for i, model in enumerate(models, 1):
        print(f"  {i}. {model['name']}")
    print()
    
    # Download each model
    success_count = 0
    for model in models:
        print(f"\n--- {model['name']} ---")
        
        # Check if already exists
        model_path = funasr_dir / model['local_name']
        if model_path.exists():
            print(f"✓ Already exists: {model_path}")
            success_count += 1
            continue
        
        # Try download methods
        result = download_from_modelscope(model['id'], funasr_dir)
        
        if not result:
            print("Trying Python ModelScope SDK...")
            result = download_with_python(model['id'], funasr_dir)
        
        if result:
            success_count += 1
        else:
            print(f"✗ Failed to download {model['name']}")
    
    # Summary
    print("\n" + "=" * 60)
    print(f"Download Summary: {success_count}/{len(models)} models ready")
    print("=" * 60)
    
    if success_count == len(models):
        print("\n✓ All models downloaded successfully!")
        print(f"  Models location: {funasr_dir}")
        print("\nYou can now use FunASR in your ASR node:")
        print("  env:")
        print("    ASR_ENGINE: funasr")
        print(f"    ASR_MODELS_DIR: {models_dir}")
    else:
        print("\n⚠ Some models failed to download.")
        print("\nManual download instructions:")
        print("1. Visit https://modelscope.cn/models")
        print("2. Search for the model names above")
        print("3. Download and extract to:")
        print(f"   {funasr_dir}")
        sys.exit(1)


if __name__ == "__main__":
    main()