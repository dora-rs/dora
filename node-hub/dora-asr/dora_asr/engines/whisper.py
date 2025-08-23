"""
Whisper ASR engine using pywhispercpp.
"""

from typing import Optional, Dict, Any
import numpy as np

try:
    from pywhispercpp.model import Model
    WHISPER_AVAILABLE = True
except ImportError:
    WHISPER_AVAILABLE = False
    print("Warning: pywhispercpp not available. Install with: pip install pywhispercpp")

from .base import ASRInterface
from ..utils import ensure_minimum_audio_duration, detect_language_from_text
from ..config import ASRConfig


class WhisperEngine(ASRInterface):
    """Whisper ASR engine using C++ implementation"""
    
    supported_languages = ['en', 'zh', 'auto']
    
    def __init__(self):
        super().__init__()
        self.config = ASRConfig()
        
    def setup(self, **kwargs) -> None:
        """
        Setup Whisper model.
        
        Args:
            model: Model size (small, medium, large)
            models_dir: Directory to store models
        """
        if not WHISPER_AVAILABLE:
            raise ImportError("pywhispercpp is not installed")
        
        model_name = kwargs.get('model', self.config.WHISPER_MODEL)
        models_dir = kwargs.get('models_dir', self.config.get_models_dir() / "whisper")
        
        # Map model names to quantized versions
        model_mapping = {
            'small': 'small-q5_1',
            'medium': 'medium-q5_0',
            'large': 'large-v3-turbo-q5_0',
            'medium-q5_0': 'medium-q5_0',
            'large-v3-turbo-q5_0': 'large-v3-turbo-q5_0'
        }
        
        model_name = model_mapping.get(model_name, model_name)
        
        print(f"Loading Whisper model: {model_name}")
        print(f"Models directory: {models_dir}")
        
        try:
            self.model = Model(model=model_name, models_dir=str(models_dir))
            self.is_initialized = True
            print(f"Whisper model loaded successfully")
        except Exception as e:
            print(f"Failed to load Whisper model: {e}")
            raise
    
    def warmup(self) -> None:
        """Warmup Whisper model"""
        if not self.is_initialized:
            return
        
        print("Warming up Whisper model...")
        try:
            # Use a simple transcribe call for warmup
            self.model.transcribe(
                self.warmup_audiodata,
                language='en',
                print_progress=False
            )
            print("Whisper model warmed up")
        except Exception as e:
            print(f"Whisper warmup failed: {e}")
    
    def transcribe(
        self,
        audio_array: np.ndarray,
        language: Optional[str] = None,
        **kwargs
    ) -> Dict[str, Any]:
        """
        Transcribe audio using Whisper.
        
        Args:
            audio_array: Audio data (16kHz, float32)
            language: Language hint ('zh', 'en', 'auto')
            
        Returns:
            Transcription results
        """
        if not self.is_initialized:
            raise RuntimeError("Whisper model not initialized")
        
        # Ensure minimum duration
        audio_array = ensure_minimum_audio_duration(
            audio_array,
            sample_rate=self.config.SAMPLE_RATE,
            min_duration=self.config.MIN_AUDIO_DURATION
        )
        
        # Set language and prompt
        if language == 'zh':
            prompt = "以下是简体中文普通话的句子。"
            lang_code = 'zh'
        elif language == 'en':
            prompt = "The following is an English sentence."
            lang_code = 'en'
        else:
            # For auto-detection, use empty string instead of None
            prompt = ""
            lang_code = ""  # Empty string for auto-detect
        
        # Transcribe
        try:
            # Build transcribe kwargs
            transcribe_kwargs = {
                'print_progress': False
            }
            
            # Only add language if specified
            if lang_code:
                transcribe_kwargs['language'] = lang_code
            
            # Only add prompt if provided
            if prompt:
                transcribe_kwargs['initial_prompt'] = prompt
            
            segments = self.model.transcribe(
                audio_array,
                **transcribe_kwargs
            )
            
            # Combine segments
            text_parts = []
            all_segments = []
            
            for segment in segments:
                text_parts.append(segment.text)
                all_segments.append({
                    'text': segment.text,
                    'start': getattr(segment, 't0', 0) / 100,  # Convert to seconds
                    'end': getattr(segment, 't1', 0) / 100
                })
            
            full_text = ' '.join(text_parts).strip()
            
            # Detect language if auto
            detected_language = language
            if language == 'auto' or language is None:
                detected_language = detect_language_from_text(full_text)
            
            return {
                'text': full_text,
                'language': detected_language,
                'segments': all_segments if kwargs.get('return_segments', False) else None,
                'confidence': None  # Whisper doesn't provide confidence scores
            }
            
        except Exception as e:
            print(f"Whisper transcription error: {e}")
            return {
                'text': '',
                'language': 'unknown',
                'segments': None,
                'confidence': 0.0
            }