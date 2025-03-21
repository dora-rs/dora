import os
import requests
import hashlib
from pathlib import Path
import shutil

BASE_URL = "https://raw.githubusercontent.com/robotlearning123/dual_ur5_husky_mujoco/dual_ur5_husky_mujoco/husky_description/meshes_mujoco"

MESH_URLS = {
    "base_link.stl": f"{BASE_URL}/base_link.stl",
    "wheel.stl": f"{BASE_URL}/wheel.stl",
    "top_plate.stl": f"{BASE_URL}/top_plate.stl",
    "user_rail.stl": f"{BASE_URL}/user_rail.stl",
    "bumper.stl": f"{BASE_URL}/bumper.stl"
}

def get_cache_dir():
    """Get or create cache directory for mesh files"""
    cache_dir = Path.home() / ".cache" / "dora-mujoco-husky" / "meshes"
    cache_dir.mkdir(parents=True, exist_ok=True)
    return cache_dir

def download_file(url: str, filepath: Path):
    """Download file from URL with progress indicator"""
    print(f"Downloading {filepath.name} from {url}")
    try:
        response = requests.get(url, stream=True)
        response.raise_for_status()
        
        total_size = int(response.headers.get('content-length', 0))
        block_size = 8192
        
        with open(filepath, 'wb') as f:
            for chunk in response.iter_content(chunk_size=block_size):
                if chunk:
                    f.write(chunk)
        print(f"Successfully downloaded {filepath.name}")
                    
    except requests.exceptions.RequestException as e:
        print(f"Error downloading {filepath.name}: {e}")
        raise

def ensure_meshes():
    """Ensure all mesh files are available, downloading if necessary"""
    cache_dir = get_cache_dir()
    mesh_dir = Path(__file__).parent / "husky" / "meshes"
    mesh_dir.mkdir(parents=True, exist_ok=True)
    
    print("Checking mesh files...")
    for filename, url in MESH_URLS.items():
        cache_file = cache_dir / filename
        mesh_file = mesh_dir / filename
        
        # Check if file exists in cache
        if not cache_file.exists():
            download_file(url, cache_file)
        
        # Copy from cache to mesh directory if needed
        if not mesh_file.exists():
            print(f"Installing {filename} to {mesh_file}")
            shutil.copy2(cache_file, mesh_file)
    
    print("All mesh files are ready")

if __name__ == "__main__":
    ensure_meshes()