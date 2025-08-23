#!/usr/bin/env python3
"""
Test that AEC is actually capturing audio from microphone
"""

import sys
import time
import numpy as np
sys.path.insert(0, '../../node-hub/dora-aec')
from dora_aec.aec_wrapper import AECWrapper

def main():
    print("=" * 60)
    print("AEC AUDIO CAPTURE TEST")
    print("=" * 60)
    print("\nThis test captures audio from your microphone using AEC.")
    print("Try speaking, clapping, or making noise to see audio levels.\n")
    
    # Initialize AEC
    aec = AECWrapper(
        library_path='../../node-hub/dora-aec/dora_aec/lib/libAudioCapture.dylib',
        enable_aec=True,
        enable_vad=True,
        sample_rate=16000
    )
    
    print("Starting audio capture...")
    aec.start_record()
    time.sleep(1)  # Wait for initialization
    
    print("\n🎤 LISTENING - Make some noise! (Press Ctrl+C to stop)\n")
    print("Time    | Audio Level | VAD | Status")
    print("-" * 50)
    
    try:
        sample_count = 0
        total_bytes = 0
        max_amplitude_seen = 0
        
        while True:
            # Get audio data
            audio_bytes, vad_active = aec.get_audio_data()
            
            if audio_bytes:
                # Convert bytes to numpy array (int16)
                audio_array = np.frombuffer(audio_bytes, dtype=np.int16)
                
                # Calculate amplitude
                amplitude = np.abs(audio_array).max() / 32768.0
                max_amplitude_seen = max(max_amplitude_seen, amplitude)
                
                # Create level meter
                level_bar = "█" * int(amplitude * 30)
                
                # Determine status
                if amplitude > 0.1:
                    status = "🔊 LOUD!"
                elif amplitude > 0.01:
                    status = "🗣️ Speaking"
                elif amplitude > 0.001:
                    status = "🔉 Quiet"
                else:
                    status = "🔇 Silent"
                
                # Print status
                elapsed = time.time()
                print(f"{elapsed % 100:6.1f}s | {amplitude:5.3f} {level_bar:30s} | {'✓' if vad_active else ' '} | {status}")
                
                sample_count += 1
                total_bytes += len(audio_bytes)
                
                # Summary every 50 samples
                if sample_count % 50 == 0:
                    print("-" * 50)
                    print(f"Summary: {total_bytes} bytes captured, max amplitude: {max_amplitude_seen:.3f}")
                    print("-" * 50)
            
            time.sleep(0.01)  # Small delay
            
    except KeyboardInterrupt:
        print("\n\nStopping audio capture...")
        aec.stop_record()
        
        print("\nFinal Statistics:")
        print(f"  Total samples: {sample_count}")
        print(f"  Total bytes: {total_bytes}")
        print(f"  Max amplitude: {max_amplitude_seen:.3f}")
        print(f"  Average bytes/sample: {total_bytes/sample_count:.1f}" if sample_count > 0 else "")
        
        if max_amplitude_seen < 0.001:
            print("\n⚠️  WARNING: No significant audio detected!")
            print("  Check that your microphone is:")
            print("  - Connected and selected as default")
            print("  - Has permission in System Settings > Privacy > Microphone")
            print("  - Not muted")
        else:
            print("\n✅ Audio capture working correctly!")

if __name__ == "__main__":
    main()