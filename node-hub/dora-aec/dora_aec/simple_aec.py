#!/usr/bin/env python3
"""
Simplified Dora AEC Node - Event-driven, no threading
"""

import ctypes
import time
from pathlib import Path
from typing import Optional

import numpy as np
import pyarrow as pa
from dora import Node


class SimpleAECNode:
    """
    Simple event-driven AEC node without internal threading.
    Relies on Dora's event loop for all processing.
    """
    
    def __init__(self):
        self.lib_path = Path(__file__).parent / "lib" / "libAudioCapture.dylib"
        self.audio_recorder = None
        self.is_recording = False
        self.last_poll_time = 0
        self.poll_interval = 0.01  # 10ms
        
        # Try to load the library
        if self.lib_path.exists():
            try:
                self._load_library()
                self._start_recording()
            except Exception as e:
                print(f"[AEC] Failed to initialize: {e}")
                self.audio_recorder = None
        else:
            print(f"[AEC] Library not found, running in passthrough mode")
    
    def _load_library(self):
        """Load and configure the native library"""
        self.audio_recorder = ctypes.CDLL(str(self.lib_path))
        
        # Configure function signatures
        self.audio_recorder.startRecord.argtypes = []
        self.audio_recorder.startRecord.restype = None
        
        self.audio_recorder.stopRecord.argtypes = []
        self.audio_recorder.stopRecord.restype = None
        
        self.audio_recorder.getAudioData.argtypes = [
            ctypes.POINTER(ctypes.c_int),
            ctypes.POINTER(ctypes.c_bool)
        ]
        self.audio_recorder.getAudioData.restype = ctypes.POINTER(ctypes.c_ubyte)
        
        self.audio_recorder.freeAudioData.argtypes = [ctypes.POINTER(ctypes.c_ubyte)]
        self.audio_recorder.freeAudioData.restype = None
    
    def _start_recording(self):
        """Start the native audio capture"""
        if self.audio_recorder and not self.is_recording:
            self.audio_recorder.startRecord()
            self.is_recording = True
            print("[AEC] Started audio capture with echo cancellation")
    
    def _stop_recording(self):
        """Stop the native audio capture"""
        if self.audio_recorder and self.is_recording:
            self.audio_recorder.stopRecord()
            self.is_recording = False
            print("[AEC] Stopped audio capture")
    
    def poll_audio(self) -> Optional[tuple]:
        """
        Poll for audio data (non-blocking).
        Returns (audio_array, vad_status) or None if no data.
        """
        if not self.audio_recorder or not self.is_recording:
            return None
        
        # Rate limit polling
        current_time = time.time()
        if current_time - self.last_poll_time < self.poll_interval:
            return None
        self.last_poll_time = current_time
        
        # Get audio data from native library
        size = ctypes.c_int(0)
        is_voice_active = ctypes.c_bool(False)
        
        data_ptr = self.audio_recorder.getAudioData(
            ctypes.byref(size),
            ctypes.byref(is_voice_active)
        )
        
        if data_ptr and size.value > 0:
            # Convert to numpy array
            audio_bytes = bytes(data_ptr[:size.value])
            audio_int16 = np.frombuffer(audio_bytes, dtype=np.int16)
            audio_float32 = audio_int16.astype(np.float32) / 32768.0
            
            # Free the native memory
            self.audio_recorder.freeAudioData(data_ptr)
            
            return audio_float32, is_voice_active.value
        
        return None
    
    def handle_microphone(self, audio_data):
        """
        Handle external microphone input (passthrough mode).
        Used when native AEC is not available.
        """
        if self.audio_recorder:
            # Native AEC is active, ignore external input
            return None
        
        # Passthrough mode - just forward the audio
        return audio_data, True  # Assume voice is active
    
    def cleanup(self):
        """Clean up resources"""
        self._stop_recording()


def main():
    """Main entry point - pure event-driven"""
    print("[AEC] Starting Simple AEC node...")
    
    node = Node()
    aec = SimpleAECNode()
    
    try:
        while True:
            # Poll for native audio (non-blocking)
            audio_result = aec.poll_audio()
            if audio_result:
                audio_data, vad_status = audio_result
                node.send_output("audio", pa.array(audio_data))
                node.send_output("vad", pa.array([vad_status]))
            
            # Process Dora events (also non-blocking with timeout)
            event = node.next(timeout=0.001)  # 1ms timeout
            
            if event and event["type"] == "INPUT":
                input_id = event.get("id", "")
                
                if input_id == "microphone":
                    # Passthrough mode for external microphone
                    result = aec.handle_microphone(event["value"].to_numpy())
                    if result:
                        audio_data, vad_status = result
                        node.send_output("audio", pa.array(audio_data))
                        node.send_output("vad", pa.array([vad_status]))
                
                elif input_id == "control":
                    command = event["value"][0].as_py()
                    if command == "stop":
                        break
    
    except KeyboardInterrupt:
        print("[AEC] Interrupted")
    finally:
        aec.cleanup()


if __name__ == "__main__":
    main()