"""
Utility functions for ASR processing.
"""

import numpy as np
import re
from typing import Optional, Tuple


def ensure_minimum_audio_duration(
    audio_array: np.ndarray, 
    sample_rate: int = 16000,
    min_duration: float = 0.5
) -> np.ndarray:
    """
    Ensure audio has minimum duration by padding with silence.
    
    Args:
        audio_array: Input audio as numpy array
        sample_rate: Audio sample rate
        min_duration: Minimum duration in seconds
        
    Returns:
        Padded audio array
    """
    min_samples = int(min_duration * sample_rate)
    
    if len(audio_array) < min_samples:
        padding_needed = min_samples - len(audio_array)
        padding = np.zeros(padding_needed, dtype=audio_array.dtype)
        audio_array = np.concatenate([audio_array, padding])
    
    return audio_array


def detect_language_from_text(text: str) -> str:
    """
    Simple language detection based on character patterns.
    
    Args:
        text: Input text
        
    Returns:
        'zh' for Chinese, 'en' for English, 'mixed' for both
    """
    # Check for Chinese characters
    chinese_pattern = re.compile(r'[\u4e00-\u9fff]+')
    has_chinese = bool(chinese_pattern.search(text))
    
    # Check for English characters
    english_pattern = re.compile(r'[a-zA-Z]+')
    has_english = bool(english_pattern.search(text))
    
    if has_chinese and has_english:
        return 'mixed'
    elif has_chinese:
        return 'zh'
    elif has_english:
        return 'en'
    else:
        return 'unknown'


def calculate_audio_stats(audio_array: np.ndarray) -> dict:
    """
    Calculate audio statistics.
    
    Args:
        audio_array: Input audio
        
    Returns:
        Dictionary with audio statistics
    """
    return {
        'duration': len(audio_array) / 16000,  # Assuming 16kHz
        'max_amplitude': float(np.abs(audio_array).max()),
        'rms': float(np.sqrt(np.mean(audio_array**2))),
        'samples': len(audio_array)
    }


def fix_spaced_uppercase(text: str) -> str:
    """
    Fix spaced uppercase letters like " G N O M E " -> "GNOME".
    From VoiceDialogue's FunASR implementation.
    
    Args:
        text: Input text
        
    Returns:
        Fixed text
    """
    pattern = r'([A-Z])\s+([A-Z](?:\s+[A-Z])*)'
    
    def replace_func(match):
        return match.group(0).replace(' ', '')
    
    return re.sub(pattern, replace_func, text)


def normalize_transcription(text: str, language: str = 'auto') -> str:
    """
    Normalize transcription output.
    
    Args:
        text: Raw transcription
        language: Language code
        
    Returns:
        Normalized text
    """
    # Remove extra whitespace
    text = ' '.join(text.split())
    
    # Fix spaced uppercase
    text = fix_spaced_uppercase(text)
    
    # Language-specific normalization
    if language == 'zh':
        # Remove spaces between Chinese characters
        text = re.sub(r'([\u4e00-\u9fff])\s+([\u4e00-\u9fff])', r'\1\2', text)
    
    return text.strip()


def split_audio_for_long_transcription(
    audio_array: np.ndarray,
    sample_rate: int = 16000,
    chunk_duration: float = 30.0,
    overlap_duration: float = 1.0
) -> list:
    """
    Split long audio into overlapping chunks.
    
    Args:
        audio_array: Input audio
        sample_rate: Sample rate
        chunk_duration: Duration of each chunk in seconds
        overlap_duration: Overlap between chunks in seconds
        
    Returns:
        List of audio chunks with metadata
    """
    chunk_samples = int(chunk_duration * sample_rate)
    overlap_samples = int(overlap_duration * sample_rate)
    step_samples = chunk_samples - overlap_samples
    
    chunks = []
    for i in range(0, len(audio_array), step_samples):
        chunk = audio_array[i:i + chunk_samples]
        if len(chunk) < sample_rate:  # Skip very short final chunk
            break
        chunks.append({
            'audio': chunk,
            'start_time': i / sample_rate,
            'end_time': (i + len(chunk)) / sample_rate
        })
    
    return chunks


def merge_transcription_chunks(chunks: list, overlap_duration: float = 1.0) -> str:
    """
    Merge transcription chunks, handling overlaps.
    
    Args:
        chunks: List of transcription chunks with timestamps
        overlap_duration: Overlap duration to handle
        
    Returns:
        Merged transcription
    """
    if not chunks:
        return ""
    
    if len(chunks) == 1:
        return chunks[0]['text']
    
    # Simple merge - could be improved with overlap detection
    merged = []
    for chunk in chunks:
        text = chunk.get('text', '').strip()
        if text:
            merged.append(text)
    
    return ' '.join(merged)