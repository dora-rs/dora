#!/usr/bin/env python3
"""
Debug AEC Node - Provides multiple signals for debugging echo cancellation
"""

import time
import threading
from pathlib import Path
from typing import Optional, Tuple

import numpy as np
import pyarrow as pa
from dora import Node
import pyaudio


class DebugAECNode:
    """
    Debug version that captures both raw and processed audio
    Allows comparison of signals before and after AEC
    """
    
    def __init__(self):
        # Setup both capture methods
        self.aec_wrapper = self._setup_aec()  # With AEC
        self.raw_capture = self._setup_raw()  # Without AEC
        
        # Reference signal tracking
        self.current_reference = None
        self.reference_lock = threading.Lock()
        
    def _setup_aec(self):
        """Setup AEC capture (macOS VoiceProcessingIO)"""
        try:
            from .aec_wrapper import AECWrapper
            lib_path = Path(__file__).parent / "lib" / "libAudioCapture.dylib"
            if lib_path.exists():
                wrapper = AECWrapper(str(lib_path))
                wrapper.start_record()
                return wrapper
        except:
            pass
        return None
    
    def _setup_raw(self):
        """Setup raw microphone capture (no processing)"""
        try:
            self.pyaudio = pyaudio.PyAudio()
            self.stream = self.pyaudio.open(
                format=pyaudio.paInt16,
                channels=1,
                rate=16000,
                input=True,
                frames_per_buffer=512
            )
            return self.stream
        except:
            return None
    
    def get_debug_signals(self) -> dict:
        """
        Get all signals for debugging
        
        Returns dict with:
        - raw_mic: Unprocessed microphone input
        - cancelled: AEC-processed audio
        - reference: Current TTS reference signal
        - estimated_echo: Difference between raw and cancelled
        """
        signals = {}
        
        # Get raw microphone (if available)
        if self.raw_capture:
            try:
                raw_bytes = self.raw_capture.read(512, exception_on_overflow=False)
                raw_int16 = np.frombuffer(raw_bytes, dtype=np.int16)
                signals["raw_mic"] = raw_int16.astype(np.float32) / 32768.0
            except:
                signals["raw_mic"] = None
        
        # Get AEC-processed audio (if available)
        if self.aec_wrapper:
            audio_data, vad = self.aec_wrapper.get_audio_data()
            if audio_data:
                aec_int16 = np.frombuffer(audio_data, dtype=np.int16)
                signals["cancelled"] = aec_int16.astype(np.float32) / 32768.0
                signals["vad"] = vad
            else:
                signals["cancelled"] = None
                signals["vad"] = False
        
        # Get reference signal (if available)
        with self.reference_lock:
            signals["reference"] = self.current_reference
        
        # Calculate estimated echo (what was removed)
        if signals.get("raw_mic") is not None and signals.get("cancelled") is not None:
            # Ensure same length
            min_len = min(len(signals["raw_mic"]), len(signals["cancelled"]))
            if min_len > 0:
                raw = signals["raw_mic"][:min_len]
                cancelled = signals["cancelled"][:min_len]
                signals["estimated_echo"] = raw - cancelled
        
        return signals
    
    def update_reference(self, reference_audio):
        """Update the reference signal from TTS"""
        with self.reference_lock:
            if isinstance(reference_audio, np.ndarray):
                self.current_reference = reference_audio
            else:
                self.current_reference = reference_audio.to_numpy()
    
    def cleanup(self):
        """Clean up resources"""
        if self.aec_wrapper:
            self.aec_wrapper.stop_record()
        if self.raw_capture:
            self.raw_capture.stop_stream()
            self.raw_capture.close()
            self.pyaudio.terminate()


def main():
    """Main entry point for debug AEC node"""
    print("[DEBUG-AEC] Starting debug AEC node...")
    
    node = Node()
    debug_aec = DebugAECNode()
    
    stats_interval = 5.0
    last_stats_time = time.time()
    frame_count = {
        "raw_mic": 0,
        "cancelled": 0,
        "reference": 0,
        "echo": 0
    }
    
    try:
        while True:
            # Get all debug signals
            signals = debug_aec.get_debug_signals()
            
            # Send each available signal
            if signals.get("raw_mic") is not None:
                node.send_output("raw_mic", pa.array(signals["raw_mic"]))
                frame_count["raw_mic"] += 1
            
            if signals.get("cancelled") is not None:
                node.send_output("cancelled", pa.array(signals["cancelled"]))
                node.send_output("vad", pa.array([signals["vad"]]))
                frame_count["cancelled"] += 1
            
            if signals.get("reference") is not None:
                node.send_output("reference", pa.array(signals["reference"]))
                frame_count["reference"] += 1
            
            if signals.get("estimated_echo") is not None:
                node.send_output("estimated_echo", pa.array(signals["estimated_echo"]))
                frame_count["echo"] += 1
            
            # Print statistics periodically
            if time.time() - last_stats_time > stats_interval:
                print(f"[DEBUG-AEC] Stats - Raw: {frame_count['raw_mic']}, "
                      f"Cancelled: {frame_count['cancelled']}, "
                      f"Reference: {frame_count['reference']}, "
                      f"Echo: {frame_count['echo']}")
                last_stats_time = time.time()
            
            # Handle events
            event = node.next(timeout=0.001)
            if event and event["type"] == "INPUT":
                if event["id"] == "reference":
                    # Update reference signal from TTS
                    debug_aec.update_reference(event["value"])
                elif event["id"] == "control":
                    cmd = event["value"][0].as_py()
                    if cmd == "stop":
                        break
    
    except KeyboardInterrupt:
        print("[DEBUG-AEC] Interrupted")
    finally:
        debug_aec.cleanup()


if __name__ == "__main__":
    main()