#!/usr/bin/env python3
"""
Alternative mac-aec implementation with timer-based audio processing
"""

import os
import time
import threading
import numpy as np
import pyarrow as pa
from dora import Node
from pathlib import Path
from .aec_wrapper import AECWrapper


class TimerBasedAECNode:
    def __init__(self, node):
        self.node = node
        self.aec = None
        self.is_active = False
        self.timer = None
        
        # Configuration
        self.sample_rate = int(os.getenv('SAMPLE_RATE', '16000'))
        self.chunk_duration = float(os.getenv('CHUNK_DURATION', '0.1'))  # 100ms chunks
        self.enable_vad = os.getenv('ENABLE_VAD', 'true').lower() == 'true'
        
        # Initialize AEC
        lib_path = Path(__file__).parent / 'lib' / 'libAudioCapture.dylib'
        self.aec = AECWrapper(
            library_path=str(lib_path),
            enable_aec=True,
            enable_vad=self.enable_vad,
            sample_rate=self.sample_rate
        )
        
    def start(self):
        """Start audio capture with timer"""
        if not self.is_active:
            self.is_active = True
            self.aec.start_record()
            print(f"[MAC-AEC] Started with {self.chunk_duration}s chunks")
            
            # Start timer-based processing
            self.schedule_next_chunk()
    
    def stop(self):
        """Stop audio capture"""
        if self.is_active:
            self.is_active = False
            if self.timer:
                self.timer.cancel()
            self.aec.stop_record()
            print("[MAC-AEC] Stopped")
    
    def schedule_next_chunk(self):
        """Schedule next audio chunk processing"""
        if self.is_active:
            self.timer = threading.Timer(self.chunk_duration, self.process_and_reschedule)
            self.timer.start()
    
    def process_and_reschedule(self):
        """Process audio and reschedule"""
        self.process_audio_chunk()
        self.schedule_next_chunk()
    
    def process_audio_chunk(self):
        """Process one audio chunk on timer"""
        if not self.aec or not self.is_active:
            return
        
        # Get audio from AEC
        audio_bytes, vad_status = self.aec.get_audio_data()
        
        if audio_bytes is None:
            # No audio yet - this is OK for first few calls
            return
        
        # Convert to float32
        audio_int16 = np.frombuffer(audio_bytes, dtype=np.int16)
        audio_float32 = audio_int16.astype(np.float32) / 32768.0
        
        # Send outputs
        try:
            self.node.send_output("audio", pa.array(audio_float32, type=pa.float32()))
            self.node.send_output("vad", pa.array([vad_status]))
            
            # Debug print every 10 chunks (1 second at 100ms chunks)
            if hasattr(self, 'chunk_count'):
                self.chunk_count += 1
            else:
                self.chunk_count = 1
                
            if self.chunk_count % 10 == 0:
                rms = np.sqrt(np.mean(audio_float32 ** 2))
                print(f"[MAC-AEC] Chunk #{self.chunk_count}: {len(audio_float32)} samples, RMS={rms:.4f}, VAD={vad_status}")
                
        except Exception as e:
            print(f"[MAC-AEC] Error sending output: {e}")


def main():
    """Main entry point with timer-based processing"""
    print("[MAC-AEC] Starting timer-based AEC node...")
    
    node = Node()
    aec_node = TimerBasedAECNode(node)
    
    # Auto-start
    if os.getenv('AUTO_START', 'true').lower() == 'true':
        aec_node.start()
    
    try:
        # Main event loop - just handle control events
        # Audio processing happens on timer
        while True:
            event = node.next(timeout=1.0)  # Longer timeout since audio is on timer
            
            if event and event["type"] == "INPUT":
                input_id = event.get("id", "")
                
                if input_id == "control":
                    command = event["value"][0].as_py()
                    if command == "start":
                        aec_node.start()
                    elif command == "stop":
                        aec_node.stop()
                        
            elif event and event["type"] == "STOP":
                break
                
    except KeyboardInterrupt:
        print("[MAC-AEC] Interrupted")
    finally:
        aec_node.stop()
        if aec_node.aec:
            aec_node.aec.cleanup()


if __name__ == "__main__":
    main()