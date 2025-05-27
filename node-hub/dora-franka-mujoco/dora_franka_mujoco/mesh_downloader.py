# filepath: /dora-mujoco-sim/src/dora_mujoco_sim/mesh_downloader.py
"""Mesh file downloader for MuJoCo simulation."""

from pathlib import Path
from huggingface_hub import hf_hub_download

def get_required_mesh_files():
    """Get list of required mesh files from names.txt"""
    names_file = Path(__file__).parent / "franka_emika_panda" / "assets" / "names.txt"
    
    with open(names_file, 'r') as f:
        mesh_files = [line.strip() for line in f if line.strip() and (line.strip().endswith('.stl') or line.strip().endswith('.obj'))]
    
    return mesh_files

MESH_FILES = get_required_mesh_files()
REPO_ID = "SGPatil/Mujoco_franka_meshes"
REPO_TYPE = "dataset"

def get_cache_dir():
    cache_dir = Path.home() / ".cache" / "dora-mujoco-franka" / "meshes"
    cache_dir.mkdir(parents=True, exist_ok=True)
    return cache_dir

def check_mesh_files_exist():
    """Check if all required mesh files exist locally"""
    mesh_dir = Path(__file__).parent / "franka_emika_panda" / "assets"
    
    missing_files = []
    for filename in MESH_FILES:
        mesh_file = mesh_dir / filename
        if not mesh_file.exists():
            missing_files.append(filename)
    
    if missing_files:
        print(f"Missing {len(missing_files)} mesh files")
        return False
    
    print(f"All {len(MESH_FILES)} mesh files are present")
    return True

def ensure_meshes():
    """Download and install required mesh files for the simulation if they are not already present in the local cache."""
    mesh_dir = Path(__file__).parent / "franka_emika_panda" / "assets"
    mesh_dir.mkdir(parents=True, exist_ok=True)
    
    print("Checking mesh files...")
    try:
        for filename in MESH_FILES:
            # Download file from Hugging Face Hub
            downloaded_path = hf_hub_download(
                repo_id=REPO_ID,
                filename=filename,
                repo_type=REPO_TYPE,
                cache_dir=get_cache_dir()
            )
            
            mesh_file = mesh_dir / filename
            if not mesh_file.exists():
                mesh_file.write_bytes(Path(downloaded_path).read_bytes())
                
    except Exception as e:
        print(f"Error downloading mesh files: {e}")
        raise e
    
    print("All mesh files are ready")

if __name__ == "__main__":
    ensure_meshes()