"""
Dora AEC (Acoustic Echo Cancellation) Node

Provides echo-cancelled audio capture with voice activity detection
using macOS native VoiceProcessingIO AudioUnit.
"""

__version__ = "0.1.0"

from .aec_wrapper import AECWrapper

__all__ = ["AECWrapper"]