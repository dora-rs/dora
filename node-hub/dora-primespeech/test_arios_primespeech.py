#!/usr/bin/env python3
"""Test script to verify arios primespeech works correctly"""

import os
import sys
import numpy as np

# Set the model directory
os.environ["PRIMESPEECH_MODEL_DIR"] = os.path.expanduser("~/.dora/models/primespeech")

print("=" * 60)
print("Testing Arios PrimeSpeech Node")
print("=" * 60)

# Test 1: Import check
print("\n1. Testing import...")
try:
    from dora_primespeech.moyoyo_tts_wrapper_streaming_fix import MOYOYO_AVAILABLE, StreamingMoYoYoTTSWrapper
    print(f"   ✓ Import successful")
    print(f"   ✓ MOYOYO_AVAILABLE: {MOYOYO_AVAILABLE}")
except Exception as e:
    print(f"   ✗ Import failed: {e}")
    sys.exit(1)

# Test 2: Check model path
print("\n2. Testing model path...")
model_dir = os.environ.get("PRIMESPEECH_MODEL_DIR")
if model_dir:
    model_path = os.path.expanduser(model_dir)
    if os.path.exists(model_path):
        print(f"   ✓ Model directory exists: {model_path}")
        moyoyo_path = os.path.join(model_path, "moyoyo")
        if os.path.exists(moyoyo_path):
            print(f"   ✓ MoYoYo models found: {moyoyo_path}")
        else:
            print(f"   ⚠ MoYoYo models not found at: {moyoyo_path}")
    else:
        print(f"   ✗ Model directory not found: {model_path}")
else:
    print("   ✗ PRIMESPEECH_MODEL_DIR not set")

# Test 3: TTS Wrapper initialization
print("\n3. Testing TTS wrapper initialization...")
if MOYOYO_AVAILABLE:
    try:
        def test_logger(level, msg):
            print(f"   [{level}] {msg}")
        
        wrapper = StreamingMoYoYoTTSWrapper(
            voice="doubao",
            device="cpu",
            enable_streaming=False,
            logger_func=test_logger
        )
        print("   ✓ TTS wrapper initialized successfully")
        
        # Test 4: Simple synthesis (if models are available)
        if hasattr(wrapper, 'tts') and wrapper.tts is not None:
            print("\n4. Testing synthesis...")
            try:
                sample_rate, audio = wrapper.synthesize("测试", language="zh", speed=1.0)
                if audio is not None and len(audio) > 0:
                    duration = len(audio) / sample_rate
                    print(f"   ✓ Synthesis successful: {duration:.2f}s of audio")
                else:
                    print("   ✗ Synthesis returned empty audio")
            except Exception as e:
                print(f"   ✗ Synthesis failed: {e}")
        else:
            print("\n4. Skipping synthesis test (models not loaded)")
            
    except Exception as e:
        print(f"   ✗ TTS wrapper initialization failed: {e}")
else:
    print("   ⚠ MOYOYO_AVAILABLE is False, skipping TTS tests")

print("\n" + "=" * 60)
print("Test complete!")
print("=" * 60)