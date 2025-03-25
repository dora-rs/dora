"""Mesh file downloader for MuJoCo Husky simulation."""

from pathlib import Path

from huggingface_hub import hf_hub_download

MESH_FILES = [
    "base_link.stl",
    "wheel.stl",
    "top_plate.stl",
    "user_rail.stl",
    "bumper.stl"
]

REPO_ID = "SGPatil/mujoco-husky-meshes"
REPO_TYPE = "dataset"

def get_cache_dir():
    cache_dir = Path.home() / ".cache" / "dora-mujoco-husky" / "meshes"
    cache_dir.mkdir(parents=True, exist_ok=True)
    return cache_dir

def ensure_meshes():
    """Download and install required mesh files for the Husky robot simulation if they are not already present in the local cache.
    
    Raises:
        Exception: If there is an error downloading any of the mesh files.

    """
    mesh_dir = Path(__file__).parent / "husky" / "meshes"
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
                print(f"Installing {filename} to {mesh_file}")
                mesh_file.write_bytes(Path(downloaded_path).read_bytes())
                
    except Exception as e:
        print(f"Error downloading mesh files: {e}")
        raise e
    
    print("All mesh files are ready")

if __name__ == "__main__":
    ensure_meshes()