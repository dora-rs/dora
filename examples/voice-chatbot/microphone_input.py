#!/usr/bin/env python3
"""
Dynamic microphone node for speech monitor testing.
Sends audio chunks to the speech monitor node.
"""

import sys
import time
import numpy as np
import pyarrow as pa
from dora import Node
import sounddevice as sd

# Audio configuration - Match VoiceDialogue
SAMPLE_RATE = 16000  # 16kHz (same as VoiceDialogue)
CHANNELS = 1         # Mono
CHUNK_SIZE = 1024    # 64ms chunks (same as VoiceDialogue: 1024 samples)
DTYPE = np.float32   # We use float32 for compatibility with dora nodes

# ANSI colors
GREEN = "\033[32m"
YELLOW = "\033[33m"
CYAN = "\033[36m"
RED = "\033[31m"
RESET = "\033[0m"
DIM = "\033[2m"

class MicrophoneMonitor:
    def __init__(self):
        # Clear screen first
        print("\033[2J\033[H", end="")
        print(f"{CYAN}Microphone Monitor for Speech Detection{RESET}")
        print(f"{CYAN}{'='*50}{RESET}")
        
        # Connect as dynamic node
        self.node = Node(node_id="microphone")
        print(f"{GREEN}✓ Connected as 'microphone' node{RESET}")
        
        self.chunks_sent = 0
        self.start_time = time.time()
        
    def audio_callback(self, indata, frames, time_info, status):
        """Audio callback - sends to speech monitor."""
        if status and self.chunks_sent == 0:  # Only show status on first issue
            print(f"\r{RED}Audio status: {status}{RESET}")
        
        # Get mono audio as float32
        audio_data = indata[:, 0].astype(DTYPE)
        
        # Calculate audio levels for visual feedback
        rms = np.sqrt(np.mean(audio_data**2))
        peak = np.abs(audio_data).max()
        
        # Create visual level meter
        bars = int(rms * 200)  # Scale for visibility
        bar_str = '█' * min(bars, 20) + '░' * (20 - min(bars, 20))
        
        # Color based on level
        if peak < 0.001:
            color = RED  # Too quiet
        elif peak < 0.01:
            color = YELLOW  # Low but detectable
        else:
            color = GREEN  # Good level
        
        # Send to speech monitor with metadata
        try:
            self.node.send_output(
                "audio",
                pa.array(audio_data, type=pa.float32()),
                metadata={"sample_rate": SAMPLE_RATE}
            )
            self.chunks_sent += 1
            
            # Calculate elapsed time
            elapsed = time.time() - self.start_time
            
            # Single line status that overwrites itself
            status_line = (f"\r[{bar_str}] {color}Peak: {peak:.4f} RMS: {rms:.4f}{RESET} "
                          f"| Chunks: {self.chunks_sent} | Time: {elapsed:.1f}s")
            
            # Clear to end of line to remove any leftover characters
            print(f"{status_line}\033[K", end="", flush=True)
                
        except Exception as e:
            if "Already borrowed" not in str(e):
                print(f"\r{RED}Error sending: {e}{RESET}\033[K")
    
    def run(self):
        """Run the microphone monitor."""
        print(f"\n{CYAN}Audio Configuration:{RESET}")
        print(f"  Sample rate: {SAMPLE_RATE} Hz")
        print(f"  Format: float32")
        print(f"  Channels: {CHANNELS} (mono)")
        print(f"  Chunk size: {CHUNK_SIZE} samples ({CHUNK_SIZE/SAMPLE_RATE*1000:.1f}ms)")
        
        # Get device info
        try:
            device_info = sd.query_devices(kind='input')
            print(f"  Device: {device_info['name']}")
        except:
            pass
        
        print(f"\n{GREEN}Starting microphone stream...{RESET}")
        print(f"{YELLOW}Speak to trigger speech detection{RESET}")
        print(f"Press Ctrl+C to stop\n")
        
        try:
            with sd.InputStream(
                samplerate=SAMPLE_RATE,
                channels=CHANNELS,
                dtype=DTYPE,
                blocksize=CHUNK_SIZE,
                callback=self.audio_callback
            ):
                while True:
                    time.sleep(0.1)
                    
        except KeyboardInterrupt:
            print(f"\n\n{CYAN}Session complete{RESET}")
            duration = self.chunks_sent * CHUNK_SIZE / SAMPLE_RATE
            elapsed = time.time() - self.start_time
            print(f"Total audio sent: {duration:.1f} seconds")
            print(f"Total chunks: {self.chunks_sent}")
            print(f"Session time: {elapsed:.1f} seconds")
            
        except Exception as e:
            print(f"{RED}Error: {e}{RESET}")

if __name__ == "__main__":
    mic = MicrophoneMonitor()
    mic.run()