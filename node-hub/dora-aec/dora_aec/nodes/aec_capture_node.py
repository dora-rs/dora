#!/usr/bin/env python3
"""
AEC Capture Node - Only responsible for hardware audio capture
"""

import ctypes
from pathlib import Path
import numpy as np
import pyarrow as pa
from dora import Node


class AECCaptureNode:
    """Single responsibility: Capture audio with AEC from hardware"""
    
    def __init__(self):
        self.lib_path = Path(__file__).parent.parent / "lib" / "libAudioCapture.dylib"
        self.recorder = None
        self.active = False
        self._load_library()
    
    def _load_library(self):
        if not self.lib_path.exists():
            raise FileNotFoundError(f"AEC library not found at {self.lib_path}")
        
        self.recorder = ctypes.CDLL(str(self.lib_path))
        # Setup function signatures (same as before)
        self.recorder.getAudioData.argtypes = [
            ctypes.POINTER(ctypes.c_int),
            ctypes.POINTER(ctypes.c_bool)
        ]
        self.recorder.getAudioData.restype = ctypes.POINTER(ctypes.c_ubyte)
        self.recorder.freeAudioData.argtypes = [ctypes.POINTER(ctypes.c_ubyte)]
    
    def start(self):
        if not self.active:
            self.recorder.startRecord()
            self.active = True
    
    def stop(self):
        if self.active:
            self.recorder.stopRecord()
            self.active = False
    
    def get_audio(self):
        """Non-blocking audio fetch"""
        if not self.active:
            return None, False
            
        size = ctypes.c_int(0)
        vad = ctypes.c_bool(False)
        data_ptr = self.recorder.getAudioData(ctypes.byref(size), ctypes.byref(vad))
        
        if data_ptr and size.value > 0:
            audio_bytes = bytes(data_ptr[:size.value])
            self.recorder.freeAudioData(data_ptr)
            
            # Return raw int16 data
            audio_int16 = np.frombuffer(audio_bytes, dtype=np.int16)
            return audio_int16, vad.value
        
        return None, False


def main():
    node = Node()
    capture = AECCaptureNode()
    capture.start()
    
    while True:
        # Pure polling - no blocking
        audio, vad = capture.get_audio()
        if audio is not None:
            node.send_output("raw_audio", pa.array(audio))
            node.send_output("vad_status", pa.array([vad]))
        
        # Handle control events
        event = node.next(timeout=0.005)  # 5ms
        if event and event["type"] == "INPUT":
            if event["id"] == "control":
                cmd = event["value"][0].as_py()
                if cmd == "stop":
                    capture.stop()
                    break
                elif cmd == "start":
                    capture.start()
    
    capture.stop()


if __name__ == "__main__":
    main()