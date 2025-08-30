"""
FunASR engine for Chinese speech recognition.
"""

from typing import Optional, Dict, Any
import numpy as np
import re

try:
    from funasr_onnx import SeacoParaformer, CT_Transformer
    FUNASR_AVAILABLE = True
except ImportError:
    FUNASR_AVAILABLE = False
    print("Warning: funasr-onnx not available. Install with: pip install funasr-onnx")

from .base import ASRInterface
from ..utils import ensure_minimum_audio_duration, fix_spaced_uppercase
from ..config import ASRConfig


class FunASREngine(ASRInterface):
    """FunASR engine for Chinese ASR"""
    
    supported_languages = ['zh']
    
    def __init__(self):
        super().__init__()
        self.config = ASRConfig()
        self.asr_model = None
        self.punc_model = None
    
    def setup(self, **kwargs) -> None:
        """
        Setup FunASR models.
        
        Args:
            models_dir: Directory containing FunASR models
        """
        if not FUNASR_AVAILABLE:
            raise ImportError("funasr-onnx is not installed")
        
        models_dir = kwargs.get('models_dir', self.config.get_models_dir() / "funasr")
        
        # Model paths
        asr_model_name = self.config.FUNASR_ASR_MODEL
        punc_model_name = self.config.FUNASR_PUNC_MODEL
        
        asr_model_path = models_dir / asr_model_name
        punc_model_path = models_dir / punc_model_name
        
        print(f"Loading FunASR models from: {models_dir}")
        
        # Check if models exist
        if not asr_model_path.exists():
            print(f"Warning: ASR model not found at {asr_model_path}")
            print("Please download from: https://modelscope.cn/models/damo/speech_seaco_paraformer_large")
            raise FileNotFoundError(f"ASR model not found: {asr_model_path}")
        
        try:
            # Load ASR model
            print(f"Loading ASR model: {asr_model_name}")
            self.asr_model = SeacoParaformer(str(asr_model_path), quantize=True)
            
            # Load punctuation model if enabled
            if self.config.ENABLE_PUNCTUATION and punc_model_path.exists():
                print(f"Loading punctuation model: {punc_model_name}")
                self.punc_model = CT_Transformer(str(punc_model_path), quantize=True)
            else:
                print("Punctuation model disabled or not found")
                self.punc_model = None
            
            self.is_initialized = True
            print("FunASR models loaded successfully")
            
        except Exception as e:
            print(f"Failed to load FunASR models: {e}")
            import traceback
            traceback.print_exc()
            raise RuntimeError(f"Failed to load FunASR models: {str(e)}")
    
    def warmup(self) -> None:
        """Warmup FunASR models"""
        if not self.is_initialized:
            return
        
        print("Warming up FunASR models...")
        try:
            result = self.transcribe(self.warmup_audiodata, language='zh')
            print("FunASR models warmed up")
        except Exception as e:
            print(f"FunASR warmup failed: {e}")
    
    def transcribe(
        self,
        audio_array: np.ndarray,
        language: Optional[str] = None,
        **kwargs
    ) -> Dict[str, Any]:
        """
        Transcribe audio using FunASR.
        
        Args:
            audio_array: Audio data (16kHz, float32)
            language: Language hint (should be 'zh')
            
        Returns:
            Transcription results
        """
        if not self.is_initialized:
            raise RuntimeError("FunASR models not initialized")
        
        # Ensure minimum duration
        audio_array = ensure_minimum_audio_duration(
            audio_array,
            sample_rate=self.config.SAMPLE_RATE,
            min_duration=self.config.MIN_AUDIO_DURATION
        )
        
        try:
            # Run ASR
            hotwords = kwargs.get('hotwords', '')
            segments = self.asr_model(wav_content=audio_array, hotwords=hotwords)
            
            # Process segments
            transcribed_texts = []
            all_segments = []
            
            for segment in segments:
                text = segment.get("preds", "")
                
                # Add punctuation if available
                if text and self.punc_model and self.config.ENABLE_PUNCTUATION:
                    try:
                        text, _ = self.punc_model(text)
                    except Exception as e:
                        print(f"Punctuation model failed: {e}")
                
                # Fix formatting issues
                text = fix_spaced_uppercase(text)
                transcribed_texts.append(text)
                
                # Store segment info
                all_segments.append({
                    'text': text,
                    'start': segment.get('start', 0) / 1000,  # Convert ms to seconds
                    'end': segment.get('end', 0) / 1000
                })
            
            full_text = ' '.join(transcribed_texts).strip()
            
            return {
                'text': full_text,
                'language': 'zh',
                'segments': all_segments if kwargs.get('return_segments', False) else None,
                'confidence': None  # FunASR doesn't provide overall confidence
            }
            
        except Exception as e:
            print(f"FunASR transcription error: {e}")
            return {
                'text': '',
                'language': 'zh',
                'segments': None,
                'confidence': 0.0
            }