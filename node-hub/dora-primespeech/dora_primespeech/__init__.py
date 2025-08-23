"""
Dora PrimeSpeech - GPT-SoVITS Text-to-Speech Node

A high-quality TTS node using GPT-SoVITS technology for natural speech synthesis.
"""

__version__ = "0.1.0"
__author__ = "Dora PrimeSpeech Contributors"

from .config import PrimeSpeechConfig
from .main import main

__all__ = ["PrimeSpeechConfig", "main"]