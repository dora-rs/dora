#!/usr/bin/env python3
"""
Simple microphone node for testing
"""

import time
import numpy as np
import pyarrow as pa
from dora import Node

def main():
    node = Node("microphone")
    
    print("Test microphone starting...")
    
    # Send test audio data periodically
    sample_rate = 16000
    chunk_duration = 0.1  # 100ms chunks
    chunk_size = int(sample_rate * chunk_duration)
    
    for i in range(100):  # Send 100 chunks (10 seconds)
        # Generate test audio (sine wave with some noise)
        t = np.linspace(i * chunk_duration, (i + 1) * chunk_duration, chunk_size)
        frequency = 440  # A4 note
        audio = np.sin(2 * np.pi * frequency * t) * 0.3
        audio += np.random.normal(0, 0.01, chunk_size)  # Add noise
        audio = audio.astype(np.float32)
        
        # Send audio
        node.send_output("audio", pa.array(audio))
        
        if i % 10 == 0:
            print(f"Sent chunk {i}: {len(audio)} samples")
        
        time.sleep(chunk_duration)
    
    print("Test complete")

if __name__ == "__main__":
    main()