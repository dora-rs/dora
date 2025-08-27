"""
Audio metrics collection for debugging and monitoring
"""

import numpy as np
from collections import deque
from typing import List, Optional


class AudioMetrics:
    """Collect and analyze audio metrics"""
    
    def __init__(self, window_size: int = 1000):
        self.window_size = window_size
        
        # Rolling windows
        self.rms_history = deque(maxlen=window_size)
        self.peak_history = deque(maxlen=window_size)
        self.vad_history = deque(maxlen=window_size)
        
        # Current values
        self.current_rms = 0.0
        self.current_peak = 0.0
        
        # Cumulative statistics
        self.total_samples = 0
        self.total_chunks = 0
        self.vad_positive_chunks = 0
        self.clipping_count = 0
        self.silence_count = 0
        
        # Thresholds
        self.silence_threshold = 0.001
        self.clipping_threshold = 0.99
    
    def update(self, audio: np.ndarray, vad: bool):
        """Update metrics with new audio chunk"""
        # Calculate metrics
        rms = np.sqrt(np.mean(audio**2))
        peak = np.abs(audio).max()
        
        # Update current values
        self.current_rms = rms
        self.current_peak = peak
        
        # Update history
        self.rms_history.append(rms)
        self.peak_history.append(peak)
        self.vad_history.append(vad)
        
        # Update statistics
        self.total_samples += len(audio)
        self.total_chunks += 1
        
        if vad:
            self.vad_positive_chunks += 1
        
        if peak >= self.clipping_threshold:
            self.clipping_count += 1
        
        if rms < self.silence_threshold:
            self.silence_count += 1
    
    def get_summary(self) -> dict:
        """Get summary of metrics"""
        if len(self.rms_history) == 0:
            return {
                "status": "no_data",
                "total_chunks": 0
            }
        
        rms_array = np.array(self.rms_history)
        peak_array = np.array(self.peak_history)
        vad_array = np.array(self.vad_history)
        
        return {
            # Current values
            "current_rms": self.current_rms,
            "current_peak": self.current_peak,
            
            # RMS statistics
            "rms_mean": float(np.mean(rms_array)),
            "rms_std": float(np.std(rms_array)),
            "rms_min": float(np.min(rms_array)),
            "rms_max": float(np.max(rms_array)),
            
            # Peak statistics
            "peak_mean": float(np.mean(peak_array)),
            "peak_max": float(np.max(peak_array)),
            
            # VAD statistics
            "vad_positive_rate": float(np.mean(vad_array)),
            "vad_transitions": self._count_transitions(vad_array),
            
            # Cumulative statistics
            "total_samples": self.total_samples,
            "total_chunks": self.total_chunks,
            "clipping_rate": self.clipping_count / max(self.total_chunks, 1),
            "silence_rate": self.silence_count / max(self.total_chunks, 1),
            
            # Sample rate estimation (if chunks are regular)
            "estimated_sample_rate": self._estimate_sample_rate()
        }
    
    def _count_transitions(self, vad_array: np.ndarray) -> int:
        """Count VAD state transitions"""
        if len(vad_array) < 2:
            return 0
        
        transitions = 0
        for i in range(1, len(vad_array)):
            if vad_array[i] != vad_array[i-1]:
                transitions += 1
        
        return transitions
    
    def _estimate_sample_rate(self) -> Optional[float]:
        """Estimate sample rate from chunk sizes"""
        if self.total_chunks > 0 and self.total_samples > 0:
            avg_chunk_size = self.total_samples / self.total_chunks
            # Assuming chunks come at ~50Hz (20ms intervals)
            return avg_chunk_size * 50
        return None
    
    def get_audio_quality(self) -> str:
        """Get audio quality assessment"""
        if len(self.rms_history) == 0:
            return "unknown"
        
        rms_mean = np.mean(self.rms_history)
        
        if self.clipping_count > self.total_chunks * 0.01:  # >1% clipping
            return "poor_clipping"
        elif rms_mean < 0.001:
            return "too_quiet"
        elif rms_mean > 0.5:
            return "too_loud"
        elif 0.01 < rms_mean < 0.3:
            return "good"
        else:
            return "acceptable"
    
    def reset(self):
        """Reset all metrics"""
        self.rms_history.clear()
        self.peak_history.clear()
        self.vad_history.clear()
        
        self.current_rms = 0.0
        self.current_peak = 0.0
        
        self.total_samples = 0
        self.total_chunks = 0
        self.vad_positive_chunks = 0
        self.clipping_count = 0
        self.silence_count = 0