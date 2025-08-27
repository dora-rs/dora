#!/usr/bin/env python3
"""
Test script to verify PrimeSpeech TTS is generating real voice, not tones.
Run this after fixing model issues identified by diagnose_tts.py
"""

import sys
import os
from pathlib import Path
import numpy as np
import soundfile as sf

# Set environment for testing
os.environ["USE_HUGGINGFACE_MODELS"] = "true"
os.environ["VOICE_NAME"] = "Doubao"
os.environ["LOG_LEVEL"] = "DEBUG"

# Add PrimeSpeech to path
sys.path.append("/Users/yuechen/home/arios/dora/node-hub/dora-primespeech")

def test_tts():
    """Test TTS synthesis and identify if it's real voice or tones."""
    print("=" * 60)
    print("PRIMESPEECH TTS TEST")
    print("=" * 60)
    
    try:
        from dora_primespeech.config import VOICE_CONFIGS
        from dora_primespeech.moyoyo_tts_wrapper_streaming_fix import StreamingMoYoYoTTSWrapper
        
        # Check if MoYoYo is available
        from dora_primespeech import moyoyo_tts_wrapper_streaming_fix
        if not moyoyo_tts_wrapper_streaming_fix.MOYOYO_AVAILABLE:
            print("❌ MOYOYO_AVAILABLE = False")
            print("   This means TTS will generate tones!")
            print("   Check diagnose_tts.py output for fixes.")
            return False
        
        print("✅ MoYoYo TTS module available")
        
        # Initialize TTS
        models_path = Path.home() / ".dora" / "models" / "primespeech" / "moyoyo"
        print(f"\n📁 Using models from: {models_path}")
        print(f"   Models directory exists: {models_path.exists()}")
        
        # Create TTS wrapper
        print("\n🔧 Initializing TTS wrapper...")
        tts = StreamingMoYoYoTTSWrapper(
            voice="doubao",
            device="cpu",
            models_path=models_path,
            voice_config=VOICE_CONFIGS["Doubao"]
        )
        
        if tts.tts is None:
            print("❌ TTS initialization failed (tts.tts is None)")
            print("   This will cause tone generation!")
            return False
        
        print("✅ TTS wrapper initialized successfully")
        
        # Test synthesis
        text = "你好，这是一个测试。如果你能听到清晰的中文语音，说明TTS工作正常。"
        print(f"\n🎤 Synthesizing: '{text}'")
        
        # Use streaming synthesis (same as in pipeline)
        audio_chunks = []
        sample_rate = None
        
        for sr, audio_chunk in tts.synthesize_streaming(text, language="zh", speed=1.0):
            sample_rate = sr
            audio_chunks.append(audio_chunk)
            print(f"   Received chunk: {audio_chunk.shape}")
        
        if not audio_chunks:
            print("❌ No audio chunks generated")
            return False
        
        # Combine chunks
        full_audio = np.concatenate(audio_chunks)
        print(f"\n📊 Audio statistics:")
        print(f"   Sample rate: {sample_rate} Hz")
        print(f"   Duration: {len(full_audio)/sample_rate:.2f} seconds")
        print(f"   Shape: {full_audio.shape}")
        print(f"   Min value: {full_audio.min():.4f}")
        print(f"   Max value: {full_audio.max():.4f}")
        print(f"   Mean value: {full_audio.mean():.4f}")
        print(f"   Std deviation: {full_audio.std():.4f}")
        
        # Detect if it's a tone
        is_tone = detect_tone(full_audio, sample_rate)
        
        # Save to file
        output_file = "test_tts_output.wav"
        sf.write(output_file, full_audio, sample_rate)
        print(f"\n💾 Saved to: {output_file}")
        
        if is_tone:
            print("\n⚠️  WARNING: Generated audio appears to be a tone!")
            print("   The audio has characteristics of a sine wave.")
            print("   Check diagnose_tts.py for model issues.")
            return False
        else:
            print("\n✅ SUCCESS: Generated real voice audio!")
            print("   The audio has natural speech characteristics.")
            print(f"   Play {output_file} to verify the voice quality.")
            return True
            
    except Exception as e:
        print(f"\n❌ ERROR during TTS test: {e}")
        import traceback
        traceback.print_exc()
        return False

def detect_tone(audio, sample_rate):
    """Detect if audio is a simple tone (sine wave) vs real speech."""
    # Check for sine wave characteristics
    # 1. Low standard deviation (sine waves are very regular)
    if audio.std() < 0.1:  # Very low variation
        return True
    
    # 2. Check for dominant frequency (FFT)
    try:
        from scipy import signal
        freqs, psd = signal.periodogram(audio, sample_rate)
        
        # Find peak frequency
        peak_idx = np.argmax(psd)
        peak_freq = freqs[peak_idx]
        peak_power = psd[peak_idx]
        total_power = np.sum(psd)
        
        # If one frequency dominates (>50% of power), it's likely a tone
        if peak_power / total_power > 0.5:
            print(f"\n🔊 Dominant frequency detected: {peak_freq:.1f} Hz")
            if 400 < peak_freq < 500:  # Near 440Hz
                print("   This is likely the 440Hz fallback tone!")
            return True
    except ImportError:
        # If scipy not available, use simpler check
        # Count zero crossings - tones have regular crossings
        zero_crossings = np.sum(np.diff(np.sign(audio)) != 0)
        expected_crossings = int(2 * 440 * len(audio) / sample_rate)  # For 440Hz tone
        
        if abs(zero_crossings - expected_crossings) < expected_crossings * 0.1:
            print(f"\n🔊 Regular zero crossings detected (like 440Hz tone)")
            return True
    
    return False

def main():
    print("\n🧪 TESTING PRIMESPEECH TTS\n")
    
    success = test_tts()
    
    print("\n" + "=" * 60)
    print("TEST RESULT")
    print("=" * 60)
    
    if success:
        print("\n✅ TTS is working correctly!")
        print("\n📝 Next steps:")
        print("   1. Listen to test_tts_output.wav to verify voice quality")
        print("   2. Run your pipeline: dora start examples/voice-chatbot/voice-chat-no-aec.yml")
    else:
        print("\n❌ TTS is generating tones instead of real voice!")
        print("\n📝 To fix this:")
        print("   1. Run: python diagnose_tts.py")
        print("   2. Follow the suggested fixes")
        print("   3. Run this test again")
    
    return 0 if success else 1

if __name__ == "__main__":
    sys.exit(main())