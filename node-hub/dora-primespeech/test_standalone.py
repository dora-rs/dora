#!/usr/bin/env python3
"""
Test the standalone PrimeSpeech implementation.
"""

import numpy as np
from dora_primespeech import GPTSoVITSEngine

def test_basic_synthesis():
    """Test basic TTS synthesis."""
    print("Testing standalone PrimeSpeech...")
    print("-" * 50)
    
    # Initialize engine
    engine = GPTSoVITSEngine(voice_name="default")
    print("✓ Engine initialized")
    
    # Test text
    test_texts = [
        ("Hello, this is a test.", "en"),
        ("你好，这是一个测试。", "zh"),
    ]
    
    for text, lang in test_texts:
        print(f"\nTesting: {text}")
        print(f"Language: {lang}")
        
        # Synthesize
        audio, sample_rate = engine.text_to_speech(text, language=lang)
        
        print(f"  Audio shape: {audio.shape}")
        print(f"  Sample rate: {sample_rate}")
        print(f"  Duration: {len(audio)/sample_rate:.2f}s")
        print(f"  Audio range: [{audio.min():.3f}, {audio.max():.3f}]")
        
        # Test streaming
        print("  Testing streaming...")
        chunks = list(engine.text_to_speech_streaming(text, language=lang))
        print(f"  Generated {len(chunks)} chunks")
    
    print("\n✅ All tests passed!")
    return True

def test_voice_change():
    """Test voice changing."""
    print("\nTesting voice changes...")
    print("-" * 50)
    
    engine = GPTSoVITSEngine()
    
    voices = ["default", "assistant", "narrator"]
    test_text = "Testing different voices."
    
    for voice in voices:
        engine.set_voice(voice)
        audio, sr = engine.text_to_speech(test_text)
        print(f"  {voice}: Generated {len(audio)/sr:.2f}s audio")
    
    print("✅ Voice change test passed!")
    return True

def test_pip_package():
    """Test that the package is properly installed."""
    print("\nTesting pip package structure...")
    print("-" * 50)
    
    # Test imports
    imports = [
        "from dora_primespeech import main",
        "from dora_primespeech import PrimeSpeechConfig",
        "from dora_primespeech import GPTSoVITSEngine",
        "from dora_primespeech.gpt_sovits_engine import SimplifiedGPTSoVITS",
    ]
    
    for import_str in imports:
        try:
            exec(import_str)
            print(f"  ✓ {import_str}")
        except ImportError as e:
            print(f"  ✗ {import_str}: {e}")
            return False
    
    print("✅ Package structure test passed!")
    return True

def main():
    """Run all tests."""
    print("="*60)
    print("PRIMESPEECH STANDALONE TEST SUITE")
    print("="*60)
    
    tests = [
        test_pip_package,
        test_basic_synthesis,
        test_voice_change,
    ]
    
    results = []
    for test_func in tests:
        try:
            result = test_func()
            results.append(result)
        except Exception as e:
            print(f"\n❌ Test failed: {e}")
            results.append(False)
    
    print("\n" + "="*60)
    print("TEST SUMMARY")
    print("="*60)
    
    if all(results):
        print("✅ All tests passed! PrimeSpeech is working correctly.")
        print("\nThe node is now self-contained and can be installed with:")
        print("  pip install -e node-hub/dora-primespeech")
        print("\nNo external MoYoYo dependencies required!")
    else:
        print("❌ Some tests failed. Please check the errors above.")
    
    return all(results)

if __name__ == "__main__":
    import sys
    sys.exit(0 if main() else 1)