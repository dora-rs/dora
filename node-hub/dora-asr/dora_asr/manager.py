"""
ASR engine manager for selecting and managing different ASR engines.
"""

import time
import json
from typing import Dict, Optional, Literal
import pyarrow as pa
from .engines import ASRInterface, WhisperEngine, FunASREngine
from .config import ASRConfig


class ASRManager:
    """
    Manages multiple ASR engines and routes based on language.
    """
    
    def __init__(self, node=None):
        self.config = ASRConfig()
        self.node = node  # Dora node for logging
        self._engines: Dict[str, ASRInterface] = {}
        self._initialized_engines: Dict[str, bool] = {}
        
        # Register available engines
        self._engine_classes = {
            'whisper': WhisperEngine,
            'funasr': FunASREngine
        }
        
        # Language to engine mapping
        self._language_to_engine = self.config.LANGUAGE_TO_ENGINE.copy()
    
    def send_log(self, level, message):
        """Send log message through node if available."""
        if self.node:
            # Define log level hierarchy
            LOG_LEVELS = {
                "DEBUG": 10,
                "INFO": 20,
                "WARNING": 30,
                "ERROR": 40
            }
            
            # Check if message should be logged
            if LOG_LEVELS.get(level, 0) < LOG_LEVELS.get(self.config.LOG_LEVEL, 20):
                return  # Skip messages below configured level
            
            # Format message with level prefix
            formatted_message = f"[{level}] {message}"
            
            log_data = {
                "node": "asr",
                "level": level,
                "message": formatted_message,
                "timestamp": time.time()
            }
            self.node.send_output("log", pa.array([json.dumps(log_data)]))
    
    def get_engine_for_language(self, language: str) -> str:
        """
        Get the best engine for a given language.
        
        Args:
            language: Language code ('zh', 'en', 'auto')
            
        Returns:
            Engine name
        """
        # Check if specific engine is configured
        if self.config.ASR_ENGINE != 'auto':
            return self.config.ASR_ENGINE
        
        # Use language mapping
        return self._language_to_engine.get(language, 'whisper')
    
    def get_or_create_engine(self, engine_name: str) -> ASRInterface:
        """
        Get or create an ASR engine instance.
        
        Args:
            engine_name: Name of the engine
            
        Returns:
            ASR engine instance
        """
        if engine_name not in self._engines:
            if engine_name not in self._engine_classes:
                raise ValueError(f"Unknown ASR engine: {engine_name}")
            
            # Create engine instance
            engine_class = self._engine_classes[engine_name]
            engine = engine_class()
            
            # Setup engine
            self.send_log("INFO", f"Initializing {engine_name} engine...")
            engine.setup()
            
            # Warmup if not done
            if engine_name not in self._initialized_engines:
                engine.warmup()
                self._initialized_engines[engine_name] = True
            
            self._engines[engine_name] = engine
        
        return self._engines[engine_name]
    
    def transcribe(
        self,
        audio_array,
        language: Optional[str] = None
    ) -> Dict:
        """
        Transcribe audio using appropriate engine.
        
        Args:
            audio_array: Audio data
            language: Language hint
            
        Returns:
            Transcription results
        """
        # Determine language if not specified
        if not language:
            language = self.config.LANGUAGE
        
        # If language is 'auto', first do a quick detection with Whisper
        actual_language = language
        if language == 'auto':
            self.send_log("INFO", "Auto-detecting language...")
            # Use Whisper for language detection (it's good at this)
            whisper_engine = self.get_or_create_engine('whisper')
            detection_result = whisper_engine.transcribe(audio_array, language='auto')
            detected_lang = detection_result.get('language', 'en')
            
            # Map detected language to our engine selection
            if detected_lang == 'zh' or detected_lang == 'chinese':
                actual_language = 'zh'
                self.send_log("INFO", f"Detected Chinese, using FunASR")
            else:
                actual_language = detected_lang
                self.send_log("INFO", f"Detected {detected_lang}, using Whisper")
        
        # Get appropriate engine based on actual language
        engine_name = self.get_engine_for_language(actual_language)
        
        # Handle engine availability
        if engine_name == 'funasr' and actual_language != 'zh':
            self.send_log("WARNING", f"FunASR only supports Chinese, falling back to Whisper")
            engine_name = 'whisper'
        
        # If we already detected with Whisper and it's the right engine, return that result
        if language == 'auto' and engine_name == 'whisper' and 'detection_result' in locals():
            return detection_result
        
        # Otherwise, get or create the appropriate engine
        try:
            engine = self.get_or_create_engine(engine_name)
        except Exception as e:
            self.send_log("ERROR", f"Failed to load {engine_name}, falling back to Whisper: {e}")
            engine = self.get_or_create_engine('whisper')
            engine_name = 'whisper'
        
        # Transcribe with the selected engine
        result = engine.transcribe(audio_array, language=actual_language)
        
        # Ensure language is in result
        if 'language' not in result or result['language'] == 'auto':
            result['language'] = actual_language
            
        return result
    
    def cleanup(self):
        """Cleanup all engines"""
        for engine in self._engines.values():
            engine.cleanup()
        self._engines.clear()
        self._initialized_engines.clear()