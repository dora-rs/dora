"""
Dora macOS AEC (Acoustic Echo Cancellation) Node

A high-performance audio capture node with hardware-accelerated echo cancellation
using macOS VoiceProcessingIO Audio Unit.
"""

__version__ = "0.1.0"

from .main import main

__all__ = ["main"]