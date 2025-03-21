from pathlib import Path
from huggingface_hub import hf_hub_download

MESH_FILES = [
    "base_link.stl",
    "wheel.stl",
    "top_plate.stl",
    "user_rail.stl",
    "bumper.stl"
]

# Replace with your Hugging Face username
REPO_ID = "SGPatil/mujoco-husky-meshes"
REPO_TYPE = "dataset"

def get_cache_dir():
    """Get or create cache directory for mesh files"""
    cache_dir = Path.home() / ".cache" / "dora-mujoco-husky" / "meshes"
    cache_dir.mkdir(parents=True, exist_ok=True)
    return cache_dir

def ensure_meshes():
    """Download mesh files from Hugging Face Hub if necessary"""
    mesh_dir = Path(__file__).parent / "husky" / "meshes"
    mesh_dir.mkdir(parents=True, exist_ok=True)
    
    print("Checking mesh files...")
    for filename in MESH_FILES:
        try:
            # Download file from Hugging Face Hub
            downloaded_path = hf_hub_download(
                repo_id=REPO_ID,
                filename=filename,
                repo_type=REPO_TYPE,
                cache_dir=get_cache_dir()
            )
            
            # Copy to mesh directory if needed
            mesh_file = mesh_dir / filename
            if not mesh_file.exists():
                print(f"Installing {filename} to {mesh_file}")
                mesh_file.write_bytes(Path(downloaded_path).read_bytes())
                
        except Exception as e:
            print(f"Error downloading {filename}: {e}")
            raise
    
    print("All mesh files are ready")

if __name__ == "__main__":
    ensure_meshes()