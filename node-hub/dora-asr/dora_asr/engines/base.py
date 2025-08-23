"""
Base interface for ASR engines.
"""

from abc import ABC, abstractmethod
from typing import Optional, Dict, Any
import numpy as np


class ASRInterface(ABC):
    """Abstract base class for ASR engines"""
    
    # Languages supported by this engine
    supported_languages = []
    
    def __init__(self):
        """Initialize ASR engine"""
        self.is_initialized = False
        self.model = None
    
    @abstractmethod
    def setup(self, **kwargs) -> None:
        """
        Setup the ASR model.
        
        Args:
            **kwargs: Engine-specific configuration
        """
        pass
    
    @abstractmethod
    def warmup(self) -> None:
        """
        Warmup the model with test audio.
        """
        pass
    
    @abstractmethod
    def transcribe(
        self, 
        audio_array: np.ndarray,
        language: Optional[str] = None,
        **kwargs
    ) -> Dict[str, Any]:
        """
        Transcribe audio to text.
        
        Args:
            audio_array: Audio data as numpy array
            language: Language hint ('zh', 'en', 'auto')
            **kwargs: Engine-specific parameters
            
        Returns:
            Dictionary with:
                - text: Transcribed text
                - language: Detected language
                - confidence: Confidence score (optional)
                - segments: Time-aligned segments (optional)
        """
        pass
    
    def cleanup(self) -> None:
        """
        Cleanup resources.
        """
        self.model = None
        self.is_initialized = False
    
    @property
    def warmup_audiodata(self) -> np.ndarray:
        """
        Generate warmup audio (1 second of silence).
        """
        return np.zeros(16000, dtype=np.float32)