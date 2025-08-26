#!/usr/bin/env python3
"""
Model Download Manager for Dora Nodes
Supports downloading models from HuggingFace, ModelScope, and direct URLs
with progress bars and resume capability.
"""

import os
import sys
import json
import argparse
from pathlib import Path
from typing import Optional, List, Dict
import shutil

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
    from huggingface_hub import snapshot_download, scan_cache_dir, hf_hub_download
    from huggingface_hub.utils import RepositoryNotFoundError
    HF_AVAILABLE = True
except ImportError:
    print("Installing huggingface-hub...")
    import subprocess
    subprocess.check_call([sys.executable, "-m", "pip", "install", "huggingface-hub"])
    from huggingface_hub import snapshot_download, scan_cache_dir, hf_hub_download
    from huggingface_hub.utils import RepositoryNotFoundError
    HF_AVAILABLE = True

# ModelScope (for FunASR)
try:
    from modelscope import snapshot_download as ms_snapshot_download
    MS_AVAILABLE = True
except ImportError:
    MS_AVAILABLE = False
    print("ModelScope not installed. Install with: pip install modelscope")


class ModelDownloader:
    """Download and manage models for Dora nodes."""
    
    # Model configurations
    MODEL_CONFIGS = {
        # Qwen MLX models
        "qwen3-8b-mlx-4bit": {
            "repo_id": "Qwen/Qwen3-8B-MLX-4bit",
            "type": "mlx",
            "size": "~4.5GB"
        },
        "qwen3-8b-mlx-8bit": {
            "repo_id": "Qwen/Qwen3-8B-MLX-8bit",
            "type": "mlx",
            "size": "~8GB"
        },
        "qwen3-14b-mlx-4bit": {
            "repo_id": "Qwen/Qwen3-14B-MLX-4bit",
            "type": "mlx",
            "size": "~7.5GB"
        },
        
        # Qwen GGUF models
        "qwen3-8b-gguf": {
            "repo_id": "Qwen/Qwen3-8B-GGUF",
            "type": "gguf",
            "files": ["Qwen3-8B-Q4_K_M.gguf"],
            "size": "~4.5GB"
        },
        
        # Whisper models
        "whisper-large-v3": {
            "repo_id": "openai/whisper-large-v3",
            "type": "whisper",
            "size": "~3GB"
        },
        "whisper-medium": {
            "repo_id": "openai/whisper-medium",
            "type": "whisper",
            "size": "~1.5GB"
        },
        "whisper-small": {
            "repo_id": "openai/whisper-small",
            "type": "whisper",
            "size": "~500MB"
        },
        "whisper-base": {
            "repo_id": "openai/whisper-base",
            "type": "whisper",
            "size": "~150MB"
        },
        "whisper-tiny": {
            "repo_id": "openai/whisper-tiny",
            "type": "whisper",
            "size": "~40MB"
        },
        
        # Distil-Whisper models
        "distil-whisper-large-v3": {
            "repo_id": "distil-whisper/distil-large-v3",
            "type": "whisper",
            "size": "~1.5GB"
        },
        
        # FunASR models (ModelScope)
        "funasr-paraformer": {
            "repo_id": "iic/speech_paraformer-large_asr_nat-zh-cn-16k-common-vocab8404-pytorch",
            "type": "funasr",
            "source": "modelscope",
            "size": "~200MB"
        },
        "funasr-punctuation": {
            "repo_id": "iic/punc_ct-transformer_zh-cn-common-vocab272727-pytorch",
            "type": "funasr",
            "source": "modelscope",
            "size": "~230MB"
        },
        
        # PrimeSpeech TTS models
        "primespeech-base": {
            "repo_id": "dora-rs/PrimeSpeech",
            "type": "primespeech",
            "files": ["model_v3.onnx", "voices/*.npz"],
            "size": "~500MB"
        }
    }
    
    def __init__(self):
        """Initialize the downloader."""
        self.hf_cache_dir = Path.home() / ".cache" / "huggingface" / "hub"
        self.ms_cache_dir = Path.home() / ".cache" / "modelscope" / "hub"
        
    def list_cached_models(self) -> Dict[str, List[str]]:
        """List all locally cached models."""
        cached = {"huggingface": [], "modelscope": []}
        
        # List HuggingFace models
        if HF_AVAILABLE and self.hf_cache_dir.exists():
            print("\n📦 Scanning HuggingFace cache...")
            cache_info = scan_cache_dir(self.hf_cache_dir)
            
            for repo in cache_info.repos:
                size_gb = repo.size_on_disk / (1024**3)
                model_info = {
                    "repo_id": repo.repo_id,
                    "size": f"{size_gb:.2f}GB",
                    "last_accessed": str(repo.last_accessed) if hasattr(repo, 'last_accessed') else "N/A",
                    "refs": [str(ref) for ref in repo.refs] if hasattr(repo, 'refs') else []
                }
                cached["huggingface"].append(model_info)
                
        # List ModelScope models
        if MS_AVAILABLE and self.ms_cache_dir.exists():
            print("\n📦 Scanning ModelScope cache...")
            # ModelScope structure can be: hub/models/owner/model_name or hub/owner/model_name
            for path in self.ms_cache_dir.rglob("*"):
                if path.is_dir() and path.name not in ["._____temp", "hub", "models"]:
                    # Check if this looks like a model directory (contains model files)
                    model_files = list(path.glob("*.bin")) + list(path.glob("*.onnx")) + \
                                 list(path.glob("*.pb")) + list(path.glob("*.pt")) + \
                                 list(path.glob("*.pth")) + list(path.glob("*.json"))
                    
                    if model_files:
                        # Calculate size
                        size = sum(f.stat().st_size for f in path.rglob("*") if f.is_file())
                        size_gb = size / (1024**3)
                        
                        # Extract model ID from path
                        relative_path = path.relative_to(self.ms_cache_dir)
                        parts = relative_path.parts
                        
                        # Skip 'models' if it's the first part
                        if parts[0] == "models":
                            parts = parts[1:]
                        
                        # Construct model ID (owner/model_name)
                        if len(parts) >= 2:
                            model_id = f"{parts[0]}/{parts[1]}"
                        else:
                            model_id = str(relative_path)
                        
                        # Avoid duplicates
                        if not any(m["model_id"] == model_id for m in cached["modelscope"]):
                            cached["modelscope"].append({
                                "model_id": model_id,
                                "size": f"{size_gb:.2f}GB"
                            })
                    
        return cached
    
    def print_cached_models(self):
        """Print cached models in a nice format."""
        cached = self.list_cached_models()
        
        print("\n" + "="*60)
        print("📚 LOCALLY CACHED MODELS")
        print("="*60)
        
        # HuggingFace models - sort by organization/owner name
        if cached["huggingface"]:
            print("\n🤗 HuggingFace Models:")
            # Sort by org name (first part before /)
            sorted_models = sorted(cached["huggingface"], 
                                 key=lambda x: (x['repo_id'].split('/')[0].lower() if '/' in x['repo_id'] 
                                              else x['repo_id'].lower()))
            
            current_org = None
            for model in sorted_models:
                repo_id = model['repo_id']
                org = repo_id.split('/')[0] if '/' in repo_id else repo_id
                
                # Print org header if it changed
                if org != current_org:
                    print(f"\n  [{org}]")
                    current_org = org
                
                # Print model with indentation
                model_name = repo_id.split('/')[1] if '/' in repo_id else repo_id
                print(f"    • {model_name:<45} [{model['size']:>8}]")
                if len(model.get("refs", [])) > 1:
                    print(f"      Versions: {', '.join(model['refs'])}")
        else:
            print("\n🤗 No HuggingFace models cached")
            
        # ModelScope models - sort by organization name  
        if cached["modelscope"]:
            print("\n🔬 ModelScope Models:")
            # Sort by org name
            sorted_models = sorted(cached["modelscope"],
                                 key=lambda x: (x['model_id'].split('/')[0].lower() if '/' in x['model_id']
                                              else x['model_id'].lower()))
            
            current_org = None
            for model in sorted_models:
                model_id = model['model_id']
                org = model_id.split('/')[0] if '/' in model_id else model_id
                
                # Print org header if it changed
                if org != current_org:
                    print(f"\n  [{org}]")
                    current_org = org
                
                # Print model with indentation
                model_name = model_id.split('/')[1] if '/' in model_id else model_id
                print(f"    • {model_name:<45} [{model['size']:>8}]")
        elif MS_AVAILABLE:
            print("\n🔬 No ModelScope models cached")
            
        print("\n" + "="*60)
        
    def download_with_progress(self, repo_id: str, model_type: str = "auto", 
                             source: str = "huggingface", files: Optional[List[str]] = None):
        """Download a model with progress bar."""
        print(f"\n📥 Downloading: {repo_id}")
        print(f"   Type: {model_type}")
        print(f"   Source: {source}")
        
        try:
            if source == "modelscope":
                if not MS_AVAILABLE:
                    print("❌ ModelScope not installed. Install with: pip install modelscope")
                    return False
                    
                print("⏳ Downloading from ModelScope...")
                cache_dir = str(self.ms_cache_dir)
                
                # Check if model exists first
                model_dir = self.ms_cache_dir / "models" / repo_id.replace("/", os.sep)
                if model_dir.exists() and any(model_dir.rglob("*.bin")) or any(model_dir.rglob("*.pt")):
                    print(f"✅ Model already cached at: {model_dir}")
                    # Verify size
                    size = sum(f.stat().st_size for f in model_dir.rglob("*") if f.is_file())
                    print(f"   Total size: {size/(1024**3):.2f}GB")
                    return True
                
                print("📊 Note: ModelScope downloads may not show detailed progress")
                print("   Downloading files...")
                
                # Monitor download in a separate thread
                import threading
                import time
                
                download_complete = False
                def monitor_download():
                    """Monitor download directory size to show progress"""
                    last_size = 0
                    while not download_complete:
                        if model_dir.exists():
                            current_size = sum(f.stat().st_size for f in model_dir.rglob("*") if f.is_file())
                            if current_size > last_size:
                                size_mb = current_size / (1024**2)
                                print(f"   Downloaded: {size_mb:.1f}MB", end='\r')
                                last_size = current_size
                        time.sleep(1)
                
                # Start monitoring thread
                monitor_thread = threading.Thread(target=monitor_download, daemon=True)
                monitor_thread.start()
                
                try:
                    # Download the model
                    model_path = ms_snapshot_download(repo_id, cache_dir=cache_dir)
                    download_complete = True
                    monitor_thread.join(timeout=1)
                    
                    # Show final size
                    if Path(model_path).exists():
                        size = sum(f.stat().st_size for f in Path(model_path).rglob("*") if f.is_file())
                        print(f"\n✅ Downloaded successfully! Total size: {size/(1024**3):.2f}GB")
                        print(f"   Location: {model_path}")
                    else:
                        print(f"\n✅ Downloaded to: {model_path}")
                        
                except Exception as e:
                    download_complete = True
                    print(f"\n❌ Error downloading from ModelScope: {e}")
                    return False
                
            else:  # HuggingFace
                if files:
                    # Download specific files
                    for file in files:
                        print(f"⏳ Downloading {file}...")
                        if "*" in file:
                            # Pattern matching not directly supported, download whole repo
                            snapshot_download(
                                repo_id,
                                cache_dir=str(self.hf_cache_dir),
                                resume_download=True,
                                tqdm_class=tqdm
                            )
                            break
                        else:
                            hf_hub_download(
                                repo_id=repo_id,
                                filename=file,
                                cache_dir=str(self.hf_cache_dir),
                                resume_download=True,
                                tqdm_class=tqdm
                            )
                else:
                    # Download entire repository
                    print("⏳ Downloading from HuggingFace Hub...")
                    
                    # Enable detailed progress output
                    import logging
                    from huggingface_hub import logging as hf_logging
                    
                    # Set HF verbosity to show download progress
                    hf_logging.set_verbosity_info()
                    
                    # Use snapshot_download with progress
                    model_path = snapshot_download(
                        repo_id,
                        cache_dir=str(self.hf_cache_dir),
                        resume_download=True,
                        local_dir_use_symlinks=False,  # Download actual files
                        ignore_patterns=["*.msgpack", "*.h5", "*.ot"]  # Skip unnecessary files
                    )
                    
                    # Reset verbosity
                    hf_logging.set_verbosity_warning()
                    
                print(f"✅ Downloaded successfully!")
                
                # Verify download
                cache_path = self.hf_cache_dir / f"models--{repo_id.replace('/', '--')}"
                if cache_path.exists():
                    size = sum(f.stat().st_size for f in cache_path.rglob("*") if f.is_file())
                    print(f"   Total size: {size/(1024**3):.2f}GB")
                    
            return True
            
        except RepositoryNotFoundError:
            print(f"❌ Model not found: {repo_id}")
            return False
        except KeyboardInterrupt:
            print("\n⚠️  Download interrupted. Run again to resume.")
            return False
        except Exception as e:
            print(f"❌ Error downloading: {e}")
            return False
            
    def download_model_by_name(self, name: str):
        """Download a model by its friendly name."""
        if name not in self.MODEL_CONFIGS:
            print(f"❌ Unknown model: {name}")
            print(f"   Available models: {', '.join(self.MODEL_CONFIGS.keys())}")
            return False
            
        config = self.MODEL_CONFIGS[name]
        return self.download_with_progress(
            repo_id=config["repo_id"],
            model_type=config["type"],
            source=config.get("source", "huggingface"),
            files=config.get("files")
        )
        
    def clean_incomplete_downloads(self):
        """Clean incomplete downloads."""
        print("\n🧹 Cleaning incomplete downloads...")
        
        incomplete_count = 0
        
        # Clean HuggingFace incomplete files
        if self.hf_cache_dir.exists():
            for incomplete in self.hf_cache_dir.rglob("*.incomplete"):
                print(f"   Removing: {incomplete.name}")
                incomplete.unlink()
                incomplete_count += 1
                
        # Clean ModelScope incomplete files
        if self.ms_cache_dir.exists():
            for incomplete in self.ms_cache_dir.rglob("*.downloading"):
                print(f"   Removing: {incomplete.name}")
                incomplete.unlink()
                incomplete_count += 1
                
        if incomplete_count > 0:
            print(f"✅ Removed {incomplete_count} incomplete files")
        else:
            print("✅ No incomplete downloads found")
            
    def remove_model(self, model_identifier: str, force: bool = False):
        """Remove a cached model."""
        removed = False
        
        # Check if it's a shortcut name
        if model_identifier in self.MODEL_CONFIGS:
            repo_id = self.MODEL_CONFIGS[model_identifier]["repo_id"]
            source = self.MODEL_CONFIGS[model_identifier].get("source", "huggingface")
        else:
            repo_id = model_identifier
            # Auto-detect source
            source = "modelscope" if any(prefix in repo_id for prefix in ["iic/", "damo/", "AI-ModelScope/"]) else "huggingface"
        
        print(f"\n🗑️  Preparing to remove: {repo_id}")
        print(f"   Source: {source}")
        
        # Find the model directory
        model_path = None
        size = 0
        
        if source == "huggingface":
            # HuggingFace model path
            model_dir = self.hf_cache_dir / f"models--{repo_id.replace('/', '--')}"
            if model_dir.exists():
                model_path = model_dir
                size = sum(f.stat().st_size for f in model_dir.rglob("*") if f.is_file())
                
        elif source == "modelscope":
            # ModelScope model path
            model_dir = self.ms_cache_dir / "models" / repo_id.replace("/", os.sep)
            if model_dir.exists():
                model_path = model_dir
                size = sum(f.stat().st_size for f in model_dir.rglob("*") if f.is_file())
        
        if not model_path or not model_path.exists():
            print(f"❌ Model not found: {repo_id}")
            return False
        
        # Show model info
        size_gb = size / (1024**3)
        print(f"   Size: {size_gb:.2f}GB")
        print(f"   Path: {model_path}")
        
        # Confirm deletion
        if not force:
            response = input(f"\n⚠️  Are you sure you want to delete {repo_id}? (y/N): ")
            if response.lower() != 'y':
                print("❌ Deletion cancelled")
                return False
        
        # Delete the model
        try:
            print(f"🗑️  Deleting {repo_id}...")
            shutil.rmtree(model_path)
            print(f"✅ Successfully removed {repo_id} ({size_gb:.2f}GB freed)")
            removed = True
            
            # Also clean up any symlinks in snapshots
            if source == "huggingface":
                # HuggingFace creates symlinks in blobs directory
                blobs_dir = self.hf_cache_dir / "blobs"
                if blobs_dir.exists():
                    # Note: We keep blobs as they might be shared
                    print("   Note: Blob files kept (may be shared with other models)")
                    
        except Exception as e:
            print(f"❌ Error removing model: {e}")
            return False
            
        return removed
    
    def remove_multiple_models(self, model_list: List[str], force: bool = False):
        """Remove multiple models."""
        total_freed = 0
        removed_count = 0
        
        for model in model_list:
            # Get size before removal
            size = 0
            if model in self.MODEL_CONFIGS:
                repo_id = self.MODEL_CONFIGS[model]["repo_id"]
            else:
                repo_id = model
                
            # Calculate size
            model_dir_hf = self.hf_cache_dir / f"models--{repo_id.replace('/', '--')}"
            model_dir_ms = self.ms_cache_dir / "models" / repo_id.replace("/", os.sep)
            
            if model_dir_hf.exists():
                size = sum(f.stat().st_size for f in model_dir_hf.rglob("*") if f.is_file())
            elif model_dir_ms.exists():
                size = sum(f.stat().st_size for f in model_dir_ms.rglob("*") if f.is_file())
            
            if self.remove_model(model, force=force):
                removed_count += 1
                total_freed += size
        
        if removed_count > 0:
            total_gb = total_freed / (1024**3)
            print(f"\n📊 Summary: Removed {removed_count} models, freed {total_gb:.2f}GB")
        
        return removed_count


def main():
    parser = argparse.ArgumentParser(
        description="Download and manage models for Dora nodes",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # List cached models
  python download_models.py --list
  
  # Download a specific model
  python download_models.py --download qwen3-8b-mlx-8bit
  
  # Download by repo ID
  python download_models.py --repo Qwen/Qwen3-8B-MLX-8bit
  
  # Remove a cached model (with confirmation)
  python download_models.py --remove whisper-tiny
  python download_models.py --remove openai/whisper-base
  
  # Remove without confirmation
  python download_models.py --remove whisper-tiny --force
  
  # Remove multiple models
  python download_models.py --remove-multiple whisper-tiny whisper-base --force
  
  # Clean incomplete downloads
  python download_models.py --clean
  
Available model shortcuts:
  - qwen3-8b-mlx-4bit    : Qwen3 8B 4-bit quantized for MLX
  - qwen3-8b-mlx-8bit    : Qwen3 8B 8-bit quantized for MLX
  - qwen3-14b-mlx-4bit   : Qwen3 14B 4-bit quantized for MLX
  - qwen3-8b-gguf        : Qwen3 8B GGUF format
  - whisper-large-v3     : OpenAI Whisper Large v3
  - whisper-medium       : OpenAI Whisper Medium
  - distil-whisper-large-v3: Distil-Whisper Large v3
  - funasr-paraformer    : FunASR Paraformer (Chinese ASR)
  - primespeech-base     : PrimeSpeech TTS base model
        """
    )
    
    parser.add_argument("--list", action="store_true", 
                       help="List all cached models")
    parser.add_argument("--download", type=str,
                       help="Download model by name (e.g., qwen3-8b-mlx-8bit)")
    parser.add_argument("--repo", type=str,
                       help="Download model by repo ID (e.g., Qwen/Qwen3-8B-MLX-8bit)")
    parser.add_argument("--source", type=str, choices=["huggingface", "modelscope"],
                       default="auto", help="Specify model source (auto-detect by default)")
    parser.add_argument("--clean", action="store_true",
                       help="Clean incomplete downloads")
    parser.add_argument("--remove", type=str,
                       help="Remove a cached model by name or repo ID")
    parser.add_argument("--remove-multiple", nargs="+",
                       help="Remove multiple models (space-separated)")
    parser.add_argument("--force", action="store_true",
                       help="Skip confirmation prompt for deletion")
    parser.add_argument("--show-models", action="store_true",
                       help="Show all available model shortcuts")
    
    args = parser.parse_args()
    
    downloader = ModelDownloader()
    
    # If no args, show help
    if not any(vars(args).values()):
        parser.print_help()
        return
    
    if args.list:
        downloader.print_cached_models()
        
    if args.show_models:
        print("\n📚 Available Model Shortcuts:")
        print("="*60)
        for name, config in downloader.MODEL_CONFIGS.items():
            print(f"{name:<25} : {config['repo_id']:<45} [{config.get('size', 'N/A')}]")
        print("="*60)
        
    if args.clean:
        downloader.clean_incomplete_downloads()
        
    if args.remove:
        downloader.remove_model(args.remove, force=args.force)
        
    if args.remove_multiple:
        downloader.remove_multiple_models(args.remove_multiple, force=args.force)
        
    if args.download:
        success = downloader.download_model_by_name(args.download)
        if success:
            print("\n✨ Model ready to use!")
            
    if args.repo:
        # Determine source
        if args.source != "auto":
            source = args.source
        else:
            # Auto-detect ModelScope models based on common patterns
            source = "huggingface"
            if any(prefix in args.repo for prefix in ["iic/", "damo/", "AI-ModelScope/", "ZhipuAI/"]):
                source = "modelscope"
                print(f"📍 Auto-detected ModelScope model based on repo pattern")
        
        success = downloader.download_with_progress(args.repo, source=source)
        if success:
            print("\n✨ Model ready to use!")


if __name__ == "__main__":
    main()