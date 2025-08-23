#!/usr/bin/env python3
"""
Model download script for PrimeSpeech.
Downloads GPT-SoVITS models from HuggingFace.
"""

import sys
import argparse
from pathlib import Path
from dora_primespeech.config import PrimeSpeechConfig, VOICE_CONFIGS
from dora_primespeech.model_manager import ModelManager


def main():
    parser = argparse.ArgumentParser(description="Download GPT-SoVITS models for PrimeSpeech")
    parser.add_argument(
        "--voice",
        type=str,
        default="all",
        help=f"Voice to download (default: all). Available: {', '.join(VOICE_CONFIGS.keys())}"
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
        help="List available voices"
    )
    
    args = parser.parse_args()
    
    if args.list:
        print("\nAvailable voices:")
        print("-" * 50)
        for voice_name, config in VOICE_CONFIGS.items():
            lang = config.get("text_lang", "unknown")
            print(f"  {voice_name:15} - Language: {lang}")
        return
    
    # Get models directory
    if args.models_dir:
        models_dir = Path(args.models_dir)
    else:
        models_dir = PrimeSpeechConfig.get_models_dir()
    
    print(f"\nModels directory: {models_dir}")
    
    # Initialize model manager
    manager = ModelManager(models_dir)
    
    # Determine which voices to download
    if args.voice == "all":
        voices_to_download = list(VOICE_CONFIGS.keys())
    else:
        if args.voice not in VOICE_CONFIGS:
            print(f"\nError: Unknown voice '{args.voice}'")
            print(f"Available voices: {', '.join(VOICE_CONFIGS.keys())}")
            sys.exit(1)
        voices_to_download = [args.voice]
    
    print(f"\nVoices to download: {', '.join(voices_to_download)}")
    print("-" * 50)
    
    # Download models for each voice
    for voice_name in voices_to_download:
        voice_config = VOICE_CONFIGS[voice_name]
        
        print(f"\n[{voice_name}]")
        
        # Check if already exists
        if manager.check_models_exist(voice_name, voice_config):
            size_mb = manager.get_model_size(voice_name, voice_config)
            print(f"  ✓ Already downloaded ({size_mb:.1f} MB)")
            continue
        
        # Download models
        print(f"  Downloading from {voice_config['repository']}...")
        try:
            model_paths = manager.get_voice_model_paths(voice_name, voice_config)
            size_mb = manager.get_model_size(voice_name, voice_config)
            print(f"  ✓ Downloaded successfully ({size_mb:.1f} MB)")
        except Exception as e:
            print(f"  ✗ Failed to download: {e}")
            continue
    
    # List all available voices
    print("\n" + "=" * 50)
    print("Available voices on disk:")
    print("-" * 50)
    
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