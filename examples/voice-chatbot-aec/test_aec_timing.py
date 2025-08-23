#!/usr/bin/env python3
"""
Test AEC timing and data flow
"""

import sys
import time
import numpy as np
sys.path.insert(0, '../../node-hub/dora-aec')
from dora_aec.aec_wrapper import AECWrapper

def main():
    print("Testing AEC timing and data flow")
    print("=" * 50)
    
    aec = AECWrapper(
        library_path='../../node-hub/dora-aec/dora_aec/lib/libAudioCapture.dylib',
        enable_aec=True,
        enable_vad=True,
        sample_rate=16000
    )
    
    print("Starting recording...")
    aec.start_record()
    
    # Track timing
    first_data_time = None
    last_data_time = None
    total_samples = 0
    chunks_received = 0
    chunk_sizes = []
    
    print("\nCollecting data for 5 seconds...")
    print("Time(s) | Chunk# | Bytes | Samples | Cumulative | Rate(Hz)")
    print("-" * 60)
    
    start_time = time.time()
    
    while time.time() - start_time < 5.0:
        audio_bytes, vad = aec.get_audio_data()
        
        if audio_bytes:
            current_time = time.time()
            elapsed = current_time - start_time
            
            if first_data_time is None:
                first_data_time = current_time
                print(f"First data after {elapsed:.3f}s")
            
            last_data_time = current_time
            chunks_received += 1
            chunk_size = len(audio_bytes)
            chunk_sizes.append(chunk_size)
            
            # Convert to samples (2 bytes per sample for int16)
            num_samples = chunk_size // 2
            total_samples += num_samples
            
            # Calculate effective sample rate
            if elapsed > 0:
                effective_rate = total_samples / elapsed
            else:
                effective_rate = 0
            
            print(f"{elapsed:6.3f} | {chunks_received:6d} | {chunk_size:5d} | {num_samples:7d} | {total_samples:10d} | {effective_rate:8.0f}")
        
        time.sleep(0.001)  # 1ms sleep
    
    print("\n" + "=" * 60)
    print("Summary:")
    
    if first_data_time:
        startup_delay = first_data_time - start_time
        total_duration = last_data_time - first_data_time
        
        print(f"  Startup delay: {startup_delay:.3f}s")
        print(f"  Data collection duration: {total_duration:.3f}s")
        print(f"  Total chunks: {chunks_received}")
        print(f"  Total samples: {total_samples}")
        print(f"  Chunk sizes: min={min(chunk_sizes)}, max={max(chunk_sizes)}, avg={np.mean(chunk_sizes):.1f}")
        print(f"  Expected samples @ 16kHz for {total_duration:.1f}s: {int(16000 * total_duration)}")
        print(f"  Actual sample rate: {total_samples/total_duration:.1f} Hz")
        
        # Check for dropped samples
        expected = int(16000 * total_duration)
        difference = total_samples - expected
        if abs(difference) > 160:  # More than 10ms difference
            print(f"  ⚠️  Sample count mismatch: {difference:+d} samples ({difference/16:.1f}ms)")
        else:
            print(f"  ✓ Sample count matches expected (diff: {difference:+d})")
    else:
        print("  No data received!")
    
    aec.stop_record()
    print("\nTest complete")

if __name__ == "__main__":
    main()