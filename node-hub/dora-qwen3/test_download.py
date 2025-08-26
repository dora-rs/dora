#!/usr/bin/env python3
"""Test download with progress bar"""

from huggingface_hub import snapshot_download
from tqdm import tqdm
import sys

def download_with_progress(repo_id):
    """Download a model with visible progress"""
    print(f"\n📥 Downloading: {repo_id}")
    print("=" * 60)
    
    try:
        # This will show progress bars for each file
        path = snapshot_download(
            repo_id,
            resume_download=True,
            tqdm_class=tqdm,
            local_dir_use_symlinks=False
        )
        
        print("=" * 60)
        print(f"✅ Download complete!")
        print(f"📁 Location: {path}")
        
    except KeyboardInterrupt:
        print("\n⚠️ Download interrupted")
    except Exception as e:
        print(f"\n❌ Error: {e}")

if __name__ == "__main__":
    # Test with a small model
    repo = sys.argv[1] if len(sys.argv) > 1 else "openai/whisper-tiny"
    download_with_progress(repo)