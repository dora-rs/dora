#!/usr/bin/env python3
"""
Universal Model Downloader
Downloads models from HuggingFace Hub, including PrimeSpeech TTS models and any other HuggingFace models.
"""

import os
import sys
import argparse
from pathlib import Path
from typing import Optional, Dict, List
import json
import fnmatch

# Progress bar imports
try:
    from tqdm import tqdm
except ImportError:
    print("Installing tqdm for progress bars...")
    import subprocess
    subprocess.check_call([sys.executable, "-m", "pip", "install", "tqdm"])
    from tqdm import tqdm

# HuggingFace Hub
try:
    from huggingface_hub import snapshot_download, hf_hub_download, list_repo_files, scan_cache_dir
    from huggingface_hub.utils import RepositoryNotFoundError
    HF_AVAILABLE = True
except ImportError:
    print("Installing huggingface-hub...")
    import subprocess
    subprocess.check_call([sys.executable, "-m", "pip", "install", "huggingface-hub"])
    from huggingface_hub import snapshot_download, hf_hub_download, list_repo_files, scan_cache_dir
    from huggingface_hub.utils import RepositoryNotFoundError
    HF_AVAILABLE = True

# Try to import PrimeSpeech modules (optional)
try:
    # Add parent directory to path to import dora_primespeech
    sys.path.insert(0, str(Path(__file__).parent.parent))
    from dora_primespeech.config import PrimeSpeechConfig, VOICE_CONFIGS
    from dora_primespeech.model_manager import ModelManager
    PRIMESPEECH_AVAILABLE = True
except ImportError:
    PRIMESPEECH_AVAILABLE = False
    print("Note: PrimeSpeech modules not found. PrimeSpeech-specific features disabled.")


def list_downloaded_models():
    """List all downloaded models in HuggingFace cache."""
    print("\n📦 Scanning for downloaded models...")
    print("=" * 60)
    
    # Scan HuggingFace cache
    hf_cache_dir = Path.home() / ".cache" / "huggingface" / "hub"
    models_found = []
    
    if hf_cache_dir.exists():
        print(f"\n📁 HuggingFace Cache: {hf_cache_dir}")
        print("-" * 60)
        
        # Try using scan_cache_dir first
        try:
            cache_info = scan_cache_dir(hf_cache_dir)
            
            if cache_info.repos:
                for repo in sorted(cache_info.repos, key=lambda x: x.repo_id):
                    size_gb = repo.size_on_disk / (1024**3)
                    print(f"  📦 {repo.repo_id:40} {size_gb:8.2f} GB")
                    models_found.append(repo.repo_id)
                
        except Exception as e:
            print(f"  Note: HF scan failed ({e}), using directory scan...")
        
        # Also do a direct directory scan for models not detected by scan_cache_dir
        for item in hf_cache_dir.iterdir():
            if item.is_dir() and not item.name.startswith('.'):
                # Determine the repo ID from directory name
                if item.name.startswith("models--"):
                    # Old format: models--org--model
                    repo_id = item.name[8:].replace("--", "/")  # Remove "models--" prefix
                else:
                    # New format: org--model
                    repo_id = item.name.replace("--", "/")
                
                # Skip if already found by scan_cache_dir
                if repo_id not in models_found and not any(repo_id in found for found in models_found):
                    # Calculate size
                    try:
                        size = sum(f.stat().st_size for f in item.rglob("*") if f.is_file())
                        size_gb = size / (1024**3)
                        if size_gb > 0.001:  # Only show if it has meaningful content (>1MB)
                            print(f"  📦 {repo_id:40} {size_gb:8.2f} GB")
                            models_found.append(repo_id)
                    except:
                        pass
        
        if not models_found:
            print("  No models found in HuggingFace cache")
    else:
        print(f"  HuggingFace cache not found at {hf_cache_dir}")
    
    # Also check common model directories
    other_dirs = [
        Path.home() / ".dora" / "models",
        Path.home() / "models",
    ]
    
    for model_dir in other_dirs:
        if model_dir.exists():
            print(f"\n📁 {model_dir}")
            print("-" * 60)
            
            # Look for model files
            model_files = list(model_dir.glob("**/*.safetensors")) + \
                         list(model_dir.glob("**/*.gguf")) + \
                         list(model_dir.glob("**/*.bin")) + \
                         list(model_dir.glob("**/*.onnx")) + \
                         list(model_dir.glob("**/*.pth")) + \
                         list(model_dir.glob("**/*.ckpt"))
            
            if model_files:
                # Group by parent directory
                model_dirs = {}
                for f in model_files:
                    parent = f.parent
                    if parent not in model_dirs:
                        model_dirs[parent] = []
                    model_dirs[parent].append(f)
                
                for dir_path, files in sorted(model_dirs.items()):
                    total_size = sum(f.stat().st_size for f in files) / (1024**3)
                    rel_path = dir_path.relative_to(model_dir)
                    print(f"  📦 {str(rel_path):40} {total_size:8.2f} GB   ({len(files)} files)")
            else:
                print(f"  No model files found")
    
    # Count total unique models across all locations
    all_model_count = len(set(models_found))  # HuggingFace models
    
    # Add count from other directories (approximation based on subdirs with model files)
    for model_dir in other_dirs:
        if model_dir.exists():
            model_subdirs = set()
            for f in model_dir.glob("**/*.bin"):
                model_subdirs.add(f.parent)
            for f in model_dir.glob("**/*.safetensors"):
                model_subdirs.add(f.parent)
            for f in model_dir.glob("**/*.gguf"):
                model_subdirs.add(f.parent)
            for f in model_dir.glob("**/*.pth"):
                model_subdirs.add(f.parent)
            for f in model_dir.glob("**/*.ckpt"):
                model_subdirs.add(f.parent)
            all_model_count += len(model_subdirs)
    
    print("\n" + "=" * 60)
    print(f"Total unique models/voices: {all_model_count}")
    return models_found


def download_huggingface_model(repo_id: str, local_dir: Optional[Path] = None, 
                               patterns: Optional[List[str]] = None,
                               revision: str = "main") -> bool:
    """Download any model from HuggingFace Hub.
    
    Args:
        repo_id: Repository ID (e.g., 'mlx-community/gemma-3-12b-it-4bit')
        local_dir: Local directory to save model (default: ~/.cache/huggingface/hub/repo_name)
        patterns: File patterns to download (e.g., ['*.safetensors', '*.json'])
        revision: Git revision to download
        
    Returns:
        True if successful, False otherwise
    """
    print(f"\n📥 Downloading HuggingFace model: {repo_id}")
    
    # Determine local directory
    if local_dir is None:
        repo_name = repo_id.replace("/", "--")
        local_dir = Path.home() / ".cache" / "huggingface" / "hub" / repo_name
    
    print(f"   Destination: {local_dir}")
    
    try:
        # List files in repository
        print("   Fetching file list...")
        files = list_repo_files(repo_id, revision=revision)
        
        # Filter files if patterns provided
        if patterns:
            filtered_files = []
            for file in files:
                for pattern in patterns:
                    if fnmatch.fnmatch(file, pattern):
                        filtered_files.append(file)
                        break
            files = filtered_files
            print(f"   Files to download: {len(files)} (filtered)")
        else:
            print(f"   Files to download: {len(files)}")
        
        # Show file preview
        if len(files) > 10:
            print(f"   First 10 files: {files[:10]}")
            print(f"   ... and {len(files) - 10} more")
        else:
            for f in files:
                print(f"   - {f}")
        
        # Download using snapshot_download
        print("\n⏳ Starting download...")
        downloaded_path = snapshot_download(
            repo_id=repo_id,
            revision=revision,
            local_dir=str(local_dir),
            local_dir_use_symlinks=False,
            allow_patterns=patterns,
            resume_download=True,
            max_workers=4
        )
        
        print(f"✅ Model downloaded successfully to: {downloaded_path}")
        
        # Calculate total size
        total_size = 0
        for root, dirs, filenames in os.walk(local_dir):
            for filename in filenames:
                if not filename.startswith('.'):
                    file_path = Path(root) / filename
                    if file_path.exists():
                        total_size += file_path.stat().st_size
        
        size_gb = total_size / (1024**3)
        print(f"   Total size: {size_gb:.2f} GB")
        
        return True
        
    except RepositoryNotFoundError:
        print(f"❌ Repository not found: {repo_id}")
        print("   Please check the repository name.")
        return False
    except Exception as e:
        print(f"❌ Error downloading model: {e}")
        return False


def download_primespeech_base(models_dir: Path):
    """Download PrimeSpeech base models (Chinese Hubert and Roberta) from HuggingFace."""
    print("\n📥 Downloading PrimeSpeech base models")
    print("   Type: primespeech base (Chinese Hubert & Roberta)")
    print("   Source: MoYoYoTech/tone-models")
    
    manager = ModelManager(models_dir)
    
    # Base pretrained model files needed by all voices
    base_files = {
        'chinese-hubert-base/config.json': 'chinese-hubert-base/config.json',
        'chinese-hubert-base/preprocessor_config.json': 'chinese-hubert-base/preprocessor_config.json',
        'chinese-hubert-base/pytorch_model.bin': 'chinese-hubert-base/pytorch_model.bin',
        'chinese-roberta-wwm-ext-large/config.json': 'chinese-roberta-wwm-ext-large/config.json',
        'chinese-roberta-wwm-ext-large/pytorch_model.bin': 'chinese-roberta-wwm-ext-large/pytorch_model.bin',
        'chinese-roberta-wwm-ext-large/tokenizer.json': 'chinese-roberta-wwm-ext-large/tokenizer.json',
    }
    
    downloaded_count = 0
    for filename in base_files.keys():
        output_path = models_dir / "moyoyo" / filename
        if output_path.exists():
            print(f"✓ {filename} already exists")
            downloaded_count += 1
            continue
            
        try:
            print(f"⏳ Downloading {filename}...")
            # Use model manager to download
            manager.download_file_from_huggingface(filename)
            print(f"✅ Downloaded {filename}")
            downloaded_count += 1
        except Exception as e:
            print(f"❌ Error downloading {filename}: {e}")
    
    if downloaded_count == len(base_files):
        print("✅ All PrimeSpeech base models downloaded successfully!")
        return True
    else:
        print(f"⚠️  Downloaded {downloaded_count}/{len(base_files)} base model files")
        return downloaded_count > 0


def download_voice_models(voice_name: str, models_dir: Path):
    """Download voice-specific models."""
    manager = ModelManager(models_dir)
    
    if voice_name == "all":
        voices_to_download = list(VOICE_CONFIGS.keys())
    else:
        if voice_name not in VOICE_CONFIGS:
            print(f"Error: Unknown voice '{voice_name}'")
            print(f"Available voices: {', '.join(VOICE_CONFIGS.keys())}")
            return False
        voices_to_download = [voice_name]
    
    print(f"\nVoices to download: {', '.join(voices_to_download)}")
    print("-" * 50)
    
    for voice in voices_to_download:
        voice_config = VOICE_CONFIGS[voice]
        
        print(f"\n[{voice}]")
        
        if manager.check_models_exist(voice, voice_config):
            size_mb = manager.get_model_size(voice, voice_config)
            print(f"  ✓ Already downloaded ({size_mb:.1f} MB)")
            continue
        
        print(f"  Downloading from {voice_config['repository']}...")
        try:
            model_paths = manager.get_voice_model_paths(voice, voice_config)
            size_mb = manager.get_model_size(voice, voice_config)
            print(f"  ✓ Downloaded successfully ({size_mb:.1f} MB)")
        except Exception as e:
            print(f"  ✗ Failed to download: {e}")
    
    return True


def main():
    parser = argparse.ArgumentParser(description="Universal Model Downloader - Download models from HuggingFace Hub")
    
    # Add --download argument for compatibility
    parser.add_argument(
        "--download",
        type=str,
        help="Model to download (HuggingFace repo ID like 'mlx-community/gemma-3-12b-it-4bit' or 'primespeech-base')"
    )
    
    # HuggingFace-specific arguments
    parser.add_argument(
        "--hf-dir",
        type=str,
        default=None,
        help="Local directory for HuggingFace model (default: ~/.cache/huggingface/hub/repo_name)"
    )
    
    parser.add_argument(
        "--patterns",
        type=str,
        nargs="+",
        help="File patterns to download (e.g., '*.safetensors' '*.json')"
    )
    
    parser.add_argument(
        "--revision",
        type=str,
        default="main",
        help="Git revision to download (default: main)"
    )
    
    # Original arguments
    parser.add_argument(
        "--voice",
        type=str,
        default=None,
        help=f"Voice to download. Available: all, {', '.join(VOICE_CONFIGS.keys())}"
    )
    parser.add_argument(
        "--models-dir",
        type=str,
        default=None,
        help="Directory to store models (default: ~/.dora/models/primespeech)"
    )
    parser.add_argument(
        "--list",
        action="store_true",
        help="List all downloaded models"
    )
    
    parser.add_argument(
        "--list-voices",
        action="store_true",
        help="List available PrimeSpeech voices"
    )
    
    args = parser.parse_args()
    
    # Handle --list (show all downloaded models)
    if args.list:
        list_downloaded_models()
        return
    
    # Handle --list-voices (show PrimeSpeech voices)
    if args.list_voices:
        if PRIMESPEECH_AVAILABLE:
            print("\nAvailable PrimeSpeech voices:")
            print("-" * 50)
            for voice_name, config in VOICE_CONFIGS.items():
                lang = config.get("text_lang", "unknown")
                print(f"  {voice_name:15} - Language: {lang}")
        else:
            print("\nPrimeSpeech not available. Use --download with a HuggingFace repo ID.")
        return
    
    # Get models directory for PrimeSpeech
    if PRIMESPEECH_AVAILABLE:
        if args.models_dir:
            models_dir = Path(args.models_dir)
        else:
            models_dir = PrimeSpeechConfig.get_models_dir()
        
        # Only print for PrimeSpeech operations
        if args.voice or (args.download and args.download == "primespeech-base"):
            print(f"\nModels directory: {models_dir}")
            manager = ModelManager(models_dir)
    
    # Handle --download argument
    if args.download:
        # Check if it's a HuggingFace repo (contains '/')
        if '/' in args.download:
            # It's a HuggingFace repo ID
            local_dir = Path(args.hf_dir) if args.hf_dir else None
            success = download_huggingface_model(
                repo_id=args.download,
                local_dir=local_dir,
                patterns=args.patterns,
                revision=args.revision
            )
            if not success:
                sys.exit(1)
        elif args.download == "primespeech-base" and PRIMESPEECH_AVAILABLE:
            success = download_primespeech_base(models_dir)
            if not success:
                sys.exit(1)
        elif PRIMESPEECH_AVAILABLE:
            # Treat as voice name
            success = download_voice_models(args.download, models_dir)
            if not success:
                sys.exit(1)
        else:
            print(f"❌ Unknown model or PrimeSpeech not available: {args.download}")
            sys.exit(1)
    
    # Handle --voice argument
    elif args.voice:
        if PRIMESPEECH_AVAILABLE:
            success = download_voice_models(args.voice, models_dir)
            if not success:
                sys.exit(1)
        else:
            print("❌ PrimeSpeech not available. Cannot download voice models.")
            sys.exit(1)
    
    # Default: show help
    else:
        print("\nUsage examples:")
        print("\n  # Download any HuggingFace model:")
        print("  python download_models.py --download mlx-community/gemma-3-12b-it-4bit")
        print("  python download_models.py --download Qwen/Qwen3-8B-MLX-4bit")
        print("")
        print("  # Download with custom directory:")
        print("  python download_models.py --download mlx-community/gemma-3-12b-it-4bit --hf-dir ~/models/gemma")
        print("")
        print("  # Download only specific files:")
        print("  python download_models.py --download mlx-community/gemma-3-12b-it-4bit --patterns '*.safetensors' '*.json'")
        print("")
        print("  # List downloaded models:")
        print("  python download_models.py --list")
        
        if PRIMESPEECH_AVAILABLE:
            print("\n  # PrimeSpeech models:")
            print("  python download_models.py --download primespeech-base")
            print("  python download_models.py --voice Doubao")
            print("  python download_models.py --voice all")
            print("  python download_models.py --list-voices")
        return
    
    # Only show available voices if we downloaded something
    if args.download or args.voice:
        print("\n" + "=" * 50)
        print("Available voices on disk:")
        print("-" * 50)
        
        manager = ModelManager(models_dir)
        available = manager.list_available_voices()
        if available:
            for voice_name, metadata in available.items():
                repo = metadata.get("repository", "unknown")
                print(f"  {voice_name:15} - Repository: {repo}")
        else:
            print("  No voices found")
    
    print("\nDone!")


if __name__ == "__main__":
    main()