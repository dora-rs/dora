"""
Silero VAD wrapper - singleton pattern from VoiceDialogue.
Thread-safe implementation for real-time speech detection.
"""

from typing import Optional
import numpy as np
import torch
from silero_vad import load_silero_vad


class SileroVAD:
    """
    Thread-safe singleton Silero VAD model wrapper.
    
    Loads model once and provides speech detection for audio frames.
    """
    _instance: Optional['SileroVAD'] = None
    _model = None

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self, threshold: float = 0.7):
        """
        Initialize Silero VAD model (only on first instance).
        
        Args:
            threshold: Confidence threshold for speech detection (0.0-1.0)
        """
        if self._model is None:
            print("Initializing Silero VAD model...")
            try:
                self._model = load_silero_vad()
                self._model.reset_states()
                self.threshold = threshold
                print("Silero VAD model initialized successfully")
            except Exception as e:
                print(f"Failed to initialize Silero VAD: {e}")
                SileroVAD._instance = None
                raise

    def is_voice_active(self, audio_frame: np.ndarray, sample_rate: int = 16000) -> tuple[bool, float]:
        """
        Detect speech in audio frame.
        
        Args:
            audio_frame: Float32 numpy array [-1.0, 1.0]
            sample_rate: Must be 8000 or 16000
            
        Returns:
            (is_speech, max_probability): Speech detection result and confidence
        """
        if self._model is None:
            return False, 0.0
            
        if not isinstance(audio_frame, np.ndarray):
            return False, 0.0
            
        # Ensure float32
        if audio_frame.dtype != np.float32:
            audio_frame = audio_frame.astype(np.float32)
            
        # Window size based on sample rate
        window_size = 512 if sample_rate == 16000 else 256
        
        audio_tensor = torch.from_numpy(audio_frame)
        
        try:
            probs = []
            # Process in windows
            for i in range(0, len(audio_tensor), window_size):
                audio_slice = audio_tensor[i:i + window_size]
                if len(audio_slice) < window_size:
                    # Pad last window if needed
                    if len(audio_tensor) >= window_size:
                        audio_slice = audio_tensor[-window_size:]
                    else:
                        continue
                        
                # Get speech probability
                prob = self._model(audio_slice, sample_rate).item()
                probs.append(prob)
            
            if not probs:
                return False, 0.0
                
            max_prob = max(probs)
            is_speech = max_prob >= self.threshold
            return is_speech, max_prob
            
        except Exception as e:
            print(f"VAD detection error: {e}")
            return False, 0.0