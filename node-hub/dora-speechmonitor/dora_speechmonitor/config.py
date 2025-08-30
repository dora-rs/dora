"""
Configuration for Speech Monitor node.
Based on VoiceDialogue's SpeechMonitorConfig.
"""

import os


class SpeechMonitorConfig:
    """Speech monitoring configuration class"""
    
    # Audio thresholds
    MIN_AUDIO_AMPLITUDE = float(os.getenv("MIN_AUDIO_AMPLITUDE", "0.01"))  # Noise floor
    
    # Time thresholds (milliseconds) - from VoiceDialogue
    ACTIVE_FRAME_THRESHOLD = float(os.getenv("ACTIVE_FRAME_THRESHOLD_MS", "100"))  # 100ms
    USER_SILENCE_THRESHOLD = float(os.getenv("USER_SILENCE_THRESHOLD_MS", "1000"))  # 1s
    SILENCE_THRESHOLD = float(os.getenv("SILENCE_THRESHOLD_MS", "300"))  # 300ms
    AUDIO_FRAMES_THRESHOLD = float(os.getenv("AUDIO_FRAMES_THRESHOLD_MS", "10000"))  # 10s
    QUESTION_END_SILENCE_THRESHOLD = float(os.getenv("QUESTION_END_SILENCE_MS", "3000"))  # 3s for question end
    
    # VAD configuration
    VAD_THRESHOLD = float(os.getenv("VAD_THRESHOLD", "0.7"))  # Silero VAD confidence
    VAD_ENABLED = os.getenv("VAD_ENABLED", "true").lower() == "true"
    
    # Audio settings
    SAMPLE_RATE = int(os.getenv("SAMPLE_RATE", "16000"))  # 16kHz for Silero VAD
    
    # Queue settings
    QUEUE_TIMEOUT = float(os.getenv("QUEUE_TIMEOUT", "0.1"))  # 100ms timeout
    
    # Logging
    LOG_LEVEL = os.getenv("LOG_LEVEL", "INFO").upper()  # DEBUG, INFO, WARNING, ERROR