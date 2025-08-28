"""
Model Manager for PrimeSpeech TTS.
Handles downloading and managing GPT-SoVITS models from HuggingFace.
Based on VoiceDialogue's model management system.
"""

import os
import json
import shutil
from pathlib import Path
from typing import Dict, Optional, List
from concurrent.futures import ThreadPoolExecutor

try:
    from huggingface_hub import hf_hub_download, HfFileSystem
    HF_HUB_AVAILABLE = True
except ImportError:
    HF_HUB_AVAILABLE = False
    print("Warning: huggingface_hub not installed. Model downloading will be limited.")
    print("Install with: pip install huggingface-hub")


# Base pretrained model files that are shared across all voices
BASE_PRETRAINED_FILES = {
    'chinese-hubert-base/config.json': 'chinese-hubert-base/config.json',
    'chinese-hubert-base/preprocessor_config.json': 'chinese-hubert-base/preprocessor_config.json', 
    'chinese-hubert-base/pytorch_model.bin': 'chinese-hubert-base/pytorch_model.bin',
    'chinese-roberta-wwm-ext-large/config.json': 'chinese-roberta-wwm-ext-large/config.json',
    'chinese-roberta-wwm-ext-large/pytorch_model.bin': 'chinese-roberta-wwm-ext-large/pytorch_model.bin',
    'chinese-roberta-wwm-ext-large/tokenizer.json': 'chinese-roberta-wwm-ext-large/tokenizer.json',
}


class ModelManager:
    """Manages GPT-SoVITS model downloads and storage."""
    
    def __init__(self, models_dir: Path):
        """Initialize model manager.
        
        Args:
            models_dir: Directory to store models
        """
        self.models_dir = Path(models_dir)
        self.moyoyo_dir = self.models_dir / "moyoyo"  # MoYoYo models directory
        self.moyoyo_dir.mkdir(parents=True, exist_ok=True)
        
        # Model repository on HuggingFace
        self.repository = "MoYoYoTech/tone-models"
        
    def check_models_exist(self, voice_name: str, voice_config: Dict, verbose: bool = True) -> bool:
        """Check if model files exist locally.
        
        Args:
            voice_name: Name of the voice
            voice_config: Voice configuration dictionary
            verbose: Whether to print missing files
            
        Returns:
            True if all required model files exist
        """
        # Check voice-specific files
        required_files = [
            voice_config.get("gpt_weights", ""),
            voice_config.get("sovits_weights", ""),
            voice_config.get("reference_audio", "")
        ]
        
        for file_name in required_files:
            if file_name:
                file_path = self.moyoyo_dir / file_name
                if not file_path.exists():
                    if verbose:
                        print(f"Missing: {file_path}")
                    return False
        
        # Check base pretrained models
        for file_path in BASE_PRETRAINED_FILES.keys():
            full_path = self.moyoyo_dir / file_path
            if not full_path.exists():
                if verbose:
                    print(f"Missing base model: {full_path}")
                return False
                
        return True
    
    def download_file_from_huggingface(self, filename: str, force: bool = False) -> Path:
        """Download a single file from HuggingFace.
        
        Args:
            filename: File path in the repository
            force: Force re-download even if exists
            
        Returns:
            Path to downloaded file
        """
        output_path = self.moyoyo_dir / filename
        
        # Check if file exists and skip if not forcing
        if output_path.exists() and not force:
            return output_path
        
        # Create parent directory
        output_path.parent.mkdir(parents=True, exist_ok=True)
        
        if HF_HUB_AVAILABLE:
            try:
                print(f"Downloading: {filename}")
                hf_hub_download(
                    repo_id=self.repository,
                    filename=filename,
                    local_dir=str(self.moyoyo_dir),
                    cache_dir=str(self.moyoyo_dir / ".cache")
                )
                print(f"✓ Downloaded: {filename}")
            except Exception as e:
                print(f"Failed to download {filename}: {e}")
                # Don't raise - allow fallback to placeholder
        else:
            print(f"Cannot download {filename}: huggingface_hub not installed")
            
        return output_path
    
    def get_voice_model_paths(self, voice_name: str, voice_config: Dict) -> Dict[str, Path]:
        """Get or download model files for a voice.
        
        Args:
            voice_name: Name of the voice
            voice_config: Voice configuration dictionary
            
        Returns:
            Dictionary of model file paths
        """
        # Download models if needed
        if not self.check_models_exist(voice_name, voice_config):
            self.download_voice_models(voice_name, voice_config)
        
        # Return paths
        return {
            "gpt_model": self.moyoyo_dir / voice_config.get("gpt_weights", "GPT_weights/placeholder.ckpt"),
            "sovits_model": self.moyoyo_dir / voice_config.get("sovits_weights", "SoVITS_weights/placeholder.pth"),
            "reference_audio": self.moyoyo_dir / voice_config.get("reference_audio", "ref_audios/placeholder.wav"),
            "chinese-hubert-base": self.moyoyo_dir / "chinese-hubert-base",
            "chinese-roberta-wwm-ext-large": self.moyoyo_dir / "chinese-roberta-wwm-ext-large",
        }
    
    def download_voice_models(self, voice_name: str, voice_config: Dict):
        """Download models for a specific voice.
        
        Args:
            voice_name: Name of the voice
            voice_config: Voice configuration dictionary
        """
        print(f"Downloading models for {voice_name}...")
        
        if not HF_HUB_AVAILABLE:
            print("Warning: Creating placeholder model files (huggingface_hub not available)")
            self._create_placeholder_models(voice_name, voice_config)
            return
        
        # Files to download
        files_to_download = []
        
        # Add voice-specific files
        for key in ["gpt_weights", "sovits_weights", "reference_audio"]:
            if key in voice_config:
                files_to_download.append(voice_config[key])
        
        # Add base pretrained models
        files_to_download.extend(BASE_PRETRAINED_FILES.keys())
        
        # Try to download files (don't fail if some are missing)
        downloaded_count = 0
        for filename in files_to_download:
            if filename:
                try:
                    self.download_file_from_huggingface(filename)
                    downloaded_count += 1
                except Exception as e:
                    print(f"Skipping {filename}: {e}")
        
        # If no files were downloaded, create placeholders
        if downloaded_count == 0:
            print("No files downloaded, creating placeholders...")
            self._create_placeholder_models(voice_name, voice_config)
        else:
            print(f"✓ Downloaded {downloaded_count} files for {voice_name}")
    
    def _create_placeholder_models(self, voice_name: str, voice_config: Dict):
        """Create placeholder model files for testing.
        
        Args:
            voice_name: Name of the voice
            voice_config: Voice configuration dictionary
        """
        # Create placeholder voice files
        placeholders = [
            (voice_config.get("gpt_weights", "GPT_weights/placeholder.ckpt"), b"GPT_PLACEHOLDER"),
            (voice_config.get("sovits_weights", "SoVITS_weights/placeholder.pth"), b"SOVITS_PLACEHOLDER"),
            (voice_config.get("reference_audio", "ref_audios/placeholder.wav"), b"AUDIO_PLACEHOLDER"),
        ]
        
        for file_name, content in placeholders:
            if file_name:
                file_path = self.moyoyo_dir / file_name
                file_path.parent.mkdir(parents=True, exist_ok=True)
                if not file_path.exists():
                    file_path.write_bytes(content)
        
        # Create placeholder base models
        for file_path in BASE_PRETRAINED_FILES.keys():
            full_path = self.moyoyo_dir / file_path
            full_path.parent.mkdir(parents=True, exist_ok=True)
            if not full_path.exists():
                full_path.write_bytes(b"BASE_MODEL_PLACEHOLDER")
        
        print(f"Created placeholder models for {voice_name}")
    
    def get_model_size(self, voice_name: str, voice_config: Dict) -> float:
        """Get the total size of downloaded model files for a voice.
        
        Args:
            voice_name: Name of the voice
            voice_config: Voice configuration dictionary
            
        Returns:
            Total size in MB
        """
        total_size = 0
        
        # Check voice-specific files
        files_to_check = [
            self.moyoyo_dir / voice_config.get("gpt_weights", ""),
            self.moyoyo_dir / voice_config.get("sovits_weights", ""),
            self.moyoyo_dir / voice_config.get("reference_audio", "")
        ]
        
        for file_path in files_to_check:
            if file_path and file_path.exists():
                total_size += file_path.stat().st_size
        
        # Convert to MB
        return total_size / (1024 * 1024)
    
    def list_available_voices(self) -> Dict[str, Dict]:
        """List all downloaded voice models.
        
        Returns:
            Dictionary mapping voice names to their metadata
        """
        voices = {}
        
        # Import VOICE_CONFIGS to check against
        from .config import VOICE_CONFIGS
        
        # Check which voices have all required files downloaded
        for voice_name, voice_config in VOICE_CONFIGS.items():
            if self.check_models_exist(voice_name, voice_config, verbose=False):
                voices[voice_name] = {
                    "repository": voice_config.get("repository", self.repository),
                    "language": voice_config.get("text_lang", "unknown"),
                    "size_mb": self.get_model_size(voice_name, voice_config)
                }
        
        return voices
    
    def check_file_exists_on_huggingface(self, filename: str) -> bool:
        """Check if a file exists on HuggingFace.
        
        Args:
            filename: File path to check
            
        Returns:
            True if file exists on HuggingFace
        """
        if not HF_HUB_AVAILABLE:
            return False
            
        try:
            fs = HfFileSystem()
            remote_files = fs.ls(f'{self.repository}/{filename}')
            return len(remote_files) > 0
        except:
            return False
    
    def get_model_info(self) -> Dict:
        """Get information about available models.
        
        Returns:
            Dictionary with model information
        """
        info = {
            "repository": self.repository,
            "models_dir": str(self.moyoyo_dir),
            "downloaded_voices": self.list_available_voices(),
            "huggingface_available": HF_HUB_AVAILABLE,
        }
        
        # Check disk usage
        total_size = 0
        if self.moyoyo_dir.exists():
            for file_path in self.moyoyo_dir.rglob("*"):
                if file_path.is_file():
                    total_size += file_path.stat().st_size
        
        info["total_size_mb"] = total_size / (1024 * 1024)
        
        return info