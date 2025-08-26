"""
Dora PrimeSpeech - Standalone GPT-SoVITS Text-to-Speech Node

A self-contained TTS node using GPT-SoVITS technology without external dependencies.
"""

__version__ = "0.2.0"
__author__ = "Dora PrimeSpeech Contributors"

from .config import PrimeSpeechConfig

# Try to use standalone version first
try:
    from .main_standalone import main
    from .gpt_sovits_engine import GPTSoVITSEngine
    print("[PrimeSpeech] Using standalone implementation")
except ImportError:
    # Fallback to original if standalone not available
    from .main import main
    GPTSoVITSEngine = None
    print("[PrimeSpeech] Using original implementation")

__all__ = ["PrimeSpeechConfig", "main", "GPTSoVITSEngine"]