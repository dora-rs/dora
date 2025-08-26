#!/usr/bin/env python3
"""
Example usage of ModelManager for PrimeSpeech TTS.
Demonstrates how to download and manage GPT-SoVITS models from HuggingFace.
"""

import sys
from pathlib import Path

# Add parent directory to path to import dora_primespeech
sys.path.insert(0, str(Path(__file__).parent.parent))

from dora_primespeech.config import PrimeSpeechConfig, VOICE_CONFIGS
from dora_primespeech.model_manager import ModelManager


def main():
    """Example of using ModelManager to download and manage voice models."""
    
    # Initialize model manager with default models directory
    models_dir = PrimeSpeechConfig.get_models_dir()
    print(f"Models directory: {models_dir}\n")
    
    manager = ModelManager(models_dir)
    
    # Example 1: List all available voices from config
    print("=" * 50)
    print("Available voices in configuration:")
    print("-" * 50)
    for voice_name in VOICE_CONFIGS.keys():
        print(f"  - {voice_name}")
    print()
    
    # Example 2: Check which models are already downloaded
    print("=" * 50)
    print("Checking downloaded models:")
    print("-" * 50)
    downloaded_voices = manager.list_available_voices()
    if downloaded_voices:
        for voice_name, metadata in downloaded_voices.items():
            size_mb = metadata.get("size_mb", 0)
            lang = metadata.get("language", "unknown")
            print(f"  ✓ {voice_name:15} - {lang:5} - {size_mb:.1f} MB")
    else:
        print("  No models downloaded yet")
    print()
    
    # Example 3: Download a specific voice model
    voice_to_download = "Doubao"  # Change this to download different voices
    voice_config = VOICE_CONFIGS.get(voice_to_download)
    
    if voice_config:
        print("=" * 50)
        print(f"Downloading models for '{voice_to_download}':")
        print("-" * 50)
        
        if manager.check_models_exist(voice_to_download, voice_config):
            print(f"  ✓ Models for '{voice_to_download}' already exist")
            
            # Get model paths
            model_paths = manager.get_voice_model_paths(voice_to_download, voice_config)
            print("\n  Model paths:")
            for key, path in model_paths.items():
                if path.exists():
                    print(f"    {key}: {path}")
        else:
            print(f"  Downloading models for '{voice_to_download}'...")
            try:
                # This will download the models if they don't exist
                model_paths = manager.get_voice_model_paths(voice_to_download, voice_config)
                print(f"  ✓ Successfully downloaded models for '{voice_to_download}'")
                
                # Show downloaded files
                print("\n  Downloaded files:")
                for key, path in model_paths.items():
                    if path.exists():
                        size_mb = path.stat().st_size / (1024 * 1024)
                        print(f"    {key}: {size_mb:.1f} MB")
            except Exception as e:
                print(f"  ✗ Failed to download: {e}")
    
    # Example 4: Get model information
    print("\n" + "=" * 50)
    print("Model Manager Information:")
    print("-" * 50)
    model_info = manager.get_model_info()
    print(f"  Repository: {model_info['repository']}")
    print(f"  Models directory: {model_info['models_dir']}")
    print(f"  Total size: {model_info['total_size_mb']:.1f} MB")
    print(f"  HuggingFace available: {model_info['huggingface_available']}")
    print(f"  Downloaded voices: {len(model_info['downloaded_voices'])}")
    
    # Example 5: Download all models (uncomment to use)
    """
    print("\n" + "=" * 50)
    print("Downloading ALL voice models:")
    print("-" * 50)
    for voice_name, voice_config in VOICE_CONFIGS.items():
        if not manager.check_models_exist(voice_name, voice_config):
            print(f"\n  Downloading {voice_name}...")
            try:
                manager.get_voice_model_paths(voice_name, voice_config)
                print(f"  ✓ {voice_name} downloaded successfully")
            except Exception as e:
                print(f"  ✗ {voice_name} failed: {e}")
        else:
            print(f"  ✓ {voice_name} already exists")
    """


if __name__ == "__main__":
    main()