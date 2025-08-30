#!/usr/bin/env python3
"""
General HuggingFace Model Downloader
Downloads any model from HuggingFace Hub.
"""

import os
import sys
import argparse
from pathlib import Path

# HuggingFace Hub
try:
    from huggingface_hub import snapshot_download, hf_hub_download, list_repo_files
    from huggingface_hub.utils import RepositoryNotFoundError
except ImportError:
    print("Installing huggingface-hub...")
    import subprocess
    subprocess.check_call([sys.executable, "-m", "pip", "install", "huggingface-hub"])
    from huggingface_hub import snapshot_download, hf_hub_download, list_repo_files
    from huggingface_hub.utils import RepositoryNotFoundError

# Progress bar
try:
    from tqdm import tqdm
except ImportError:
    print("Installing tqdm for progress bars...")
    import subprocess
    subprocess.check_call([sys.executable, "-m", "pip", "install", "tqdm"])
    from tqdm import tqdm


def download_huggingface_model(repo_id: str, local_dir: Path, revision: str = "main", allow_patterns: list = None):
    """Download a model from HuggingFace Hub.
    
    Args:
        repo_id: Repository ID on HuggingFace (e.g., "mlx-community/gemma-3-12b-it-4bit")
        local_dir: Local directory to save the model
        revision: Git revision to download (default: "main")
        allow_patterns: List of file patterns to download (e.g., ["*.safetensors", "*.json"])
    """
    print(f"\n📥 Downloading model: {repo_id}")
    print(f"   Destination: {local_dir}")
    
    # Create directory if it doesn't exist
    local_dir.mkdir(parents=True, exist_ok=True)
    
    try:
        # List files in the repository
        print("   Fetching file list...")
        files = list_repo_files(repo_id, revision=revision)
        
        # Filter files if patterns provided
        if allow_patterns:
            import fnmatch
            filtered_files = []
            for file in files:
                for pattern in allow_patterns:
                    if fnmatch.fnmatch(file, pattern):
                        filtered_files.append(file)
                        break
            files = filtered_files
            print(f"   Files to download: {len(files)} (filtered)")
        else:
            print(f"   Files to download: {len(files)}")
        
        # Show some file info
        total_files = len(files)
        if total_files > 10:
            print(f"   First 10 files: {files[:10]}")
            print(f"   ... and {total_files - 10} more")
        else:
            for f in files:
                print(f"   - {f}")
        
        # Download using snapshot_download for efficiency
        print("\n⏳ Starting download...")
        downloaded_path = snapshot_download(
            repo_id=repo_id,
            revision=revision,
            local_dir=str(local_dir),
            local_dir_use_symlinks=False,
            allow_patterns=allow_patterns,
            resume_download=True,
            max_workers=4
        )
        
        print(f"✅ Model downloaded successfully to: {downloaded_path}")
        
        # Show downloaded files size
        total_size = 0
        for root, dirs, files in os.walk(local_dir):
            for file in files:
                if not file.startswith('.'):  # Skip hidden files
                    file_path = Path(root) / file
                    total_size += file_path.stat().st_size
        
        size_gb = total_size / (1024**3)
        print(f"   Total size: {size_gb:.2f} GB")
        
        return True
        
    except RepositoryNotFoundError:
        print(f"❌ Repository not found: {repo_id}")
        print("   Please check the repository name and make sure it exists on HuggingFace.")
        return False
    except Exception as e:
        print(f"❌ Error downloading model: {e}")
        return False


def main():
    parser = argparse.ArgumentParser(description="Download models from HuggingFace Hub")
    
    parser.add_argument(
        "repo_id",
        type=str,
        help="HuggingFace repository ID (e.g., mlx-community/gemma-3-12b-it-4bit)"
    )
    
    parser.add_argument(
        "--local-dir",
        type=str,
        default=None,
        help="Local directory to save the model (default: ~/.cache/huggingface/hub/<repo_name>)"
    )
    
    parser.add_argument(
        "--revision",
        type=str,
        default="main",
        help="Git revision to download (default: main)"
    )
    
    parser.add_argument(
        "--patterns",
        type=str,
        nargs="+",
        help="File patterns to download (e.g., '*.safetensors' '*.json')"
    )
    
    parser.add_argument(
        "--models-base-dir",
        type=str,
        default=None,
        help="Base directory for models (default: ~/.cache/huggingface/hub)"
    )
    
    args = parser.parse_args()
    
    # Determine local directory
    if args.local_dir:
        local_dir = Path(args.local_dir)
    else:
        # Use repo name as directory name
        repo_name = args.repo_id.replace("/", "--")
        
        if args.models_base_dir:
            base_dir = Path(args.models_base_dir)
        else:
            base_dir = Path.home() / ".cache" / "huggingface" / "hub"
        
        local_dir = base_dir / repo_name
    
    # Download the model
    success = download_huggingface_model(
        repo_id=args.repo_id,
        local_dir=local_dir,
        revision=args.revision,
        allow_patterns=args.patterns
    )
    
    if not success:
        sys.exit(1)
    
    print("\n🎉 Done!")


if __name__ == "__main__":
    main()