"""
Python wrapper for native AEC (Acoustic Echo Cancellation) library

This wrapper provides a Python interface to the macOS VoiceProcessingIO
AudioUnit through a native library.
"""

import ctypes
import threading
from typing import Optional, Tuple
from pathlib import Path


class AECWrapper:
    """
    Wrapper for native AEC library
    
    Provides thread-safe access to acoustic echo cancellation and
    voice activity detection functionality.
    """
    
    def __init__(
        self,
        library_path: str,
        enable_aec: bool = True,
        enable_vad: bool = True,
        sample_rate: int = 16000
    ):
        """
        Initialize AEC wrapper
        
        Args:
            library_path: Path to the native library (libAudioCapture.dylib)
            enable_aec: Enable acoustic echo cancellation
            enable_vad: Enable voice activity detection
            sample_rate: Audio sample rate (must be 16000 for current implementation)
        """
        self.library_path = Path(library_path)
        self.enable_aec = enable_aec
        self.enable_vad = enable_vad
        self.sample_rate = sample_rate
        
        self.audio_recorder = None
        self.is_recording = False
        self.lock = threading.Lock()
        
        # Load the native library
        self._load_library()
    
    def _load_library(self):
        """Load and configure the native library"""
        if not self.library_path.exists():
            raise FileNotFoundError(f"Native library not found at {self.library_path}")
        
        try:
            # Load the dynamic library
            self.audio_recorder = ctypes.CDLL(str(self.library_path))
            
            # Configure function signatures
            # startRecord() -> void
            self.audio_recorder.startRecord.argtypes = []
            self.audio_recorder.startRecord.restype = None
            
            # stopRecord() -> void
            self.audio_recorder.stopRecord.argtypes = []
            self.audio_recorder.stopRecord.restype = None
            
            # getAudioData(int* size, bool* isVoiceActive) -> uint8_t*
            self.audio_recorder.getAudioData.argtypes = [
                ctypes.POINTER(ctypes.c_int),
                ctypes.POINTER(ctypes.c_bool)
            ]
            self.audio_recorder.getAudioData.restype = ctypes.POINTER(ctypes.c_ubyte)
            
            # freeAudioData(uint8_t* buffer) -> void
            self.audio_recorder.freeAudioData.argtypes = [ctypes.POINTER(ctypes.c_ubyte)]
            self.audio_recorder.freeAudioData.restype = None
            
        except Exception as e:
            raise RuntimeError(f"Failed to load native library: {e}")
    
    def start_record(self):
        """Start audio recording with AEC"""
        with self.lock:
            if self.is_recording:
                return
            
            if not self.audio_recorder:
                raise RuntimeError("Native library not loaded")
            
            # Start recording in native library
            self.audio_recorder.startRecord()
            self.is_recording = True
    
    def stop_record(self):
        """Stop audio recording"""
        with self.lock:
            if not self.is_recording:
                return
            
            if not self.audio_recorder:
                return
            
            # Stop recording in native library
            self.audio_recorder.stopRecord()
            self.is_recording = False
    
    def get_audio_data(self) -> Tuple[Optional[bytes], bool]:
        """
        Get audio data from the native library
        
        Returns:
            Tuple of (audio_data, vad_status)
            - audio_data: Bytes containing audio samples (16-bit signed integers)
            - vad_status: Boolean indicating if voice is detected
            
            Returns (None, False) if no data is available
        """
        with self.lock:
            if not self.is_recording or not self.audio_recorder:
                return None, False
            
            # Prepare output parameters
            size = ctypes.c_int(0)
            is_voice_active = ctypes.c_bool(False)
            
            # Get audio data from native library
            data_ptr = self.audio_recorder.getAudioData(
                ctypes.byref(size),
                ctypes.byref(is_voice_active)
            )
            
            if data_ptr and size.value > 0:
                # Copy audio data to Python bytes
                audio_data = bytes(data_ptr[:size.value])
                
                # Free the native memory
                self.audio_recorder.freeAudioData(data_ptr)
                
                # Return audio data and VAD status
                return audio_data, is_voice_active.value if self.enable_vad else True
            else:
                # No data available
                return None, False
    
    def cleanup(self):
        """Clean up resources"""
        self.stop_record()
        self.audio_recorder = None