#!/usr/bin/env python3
"""
Standalone Dora PrimeSpeech Node - Self-contained GPT-SoVITS TTS
No external MoYoYo dependencies required.
"""

import os
import sys
import time
import json
import re
import numpy as np
import pyarrow as pa
from dora import Node
from pathlib import Path
from typing import Optional, List
import logging

# Import our self-contained engine
from .gpt_sovits_engine import GPTSoVITSEngine
from .config import PrimeSpeechConfig, VOICE_CONFIGS

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def send_log(node, level, message):
    """Send log message through log output channel."""
    LOG_LEVELS = {
        "DEBUG": 10,
        "INFO": 20,
        "WARNING": 30,
        "ERROR": 40
    }
    
    config_level = PrimeSpeechConfig.LOG_LEVEL
    if LOG_LEVELS.get(level, 0) < LOG_LEVELS.get(config_level, 20):
        return
    
    formatted_message = f"[{level}] {message}"
    log_data = {
        "node": "primespeech",
        "level": level,
        "message": formatted_message,
        "timestamp": time.time()
    }
    try:
        node.send_output("log", pa.array([json.dumps(log_data)]))
    except:
        pass  # Ignore logging errors


def segment_text_for_tts(text: str, max_length: int = 100, min_length: int = 30) -> List[str]:
    """
    Segment long text into smaller chunks for faster TTS processing.
    
    Args:
        text: Input text to segment
        max_length: Maximum characters per segment
        min_length: Minimum characters per segment
        
    Returns:
        List of text segments
    """
    if len(text) <= max_length:
        return [text]
    
    # Split by sentence delimiters
    sentence_delimiters = r'[。！？.!?；;]'
    sentences = re.split(f'({sentence_delimiters})', text)
    
    # Reconstruct sentences with their delimiters
    reconstructed = []
    for i in range(0, len(sentences), 2):
        if i + 1 < len(sentences):
            reconstructed.append(sentences[i] + sentences[i + 1])
        elif sentences[i].strip():
            reconstructed.append(sentences[i])
    
    # Group into segments
    segments = []
    current_segment = ""
    
    for sentence in reconstructed:
        if not sentence.strip():
            continue
            
        if len(current_segment) + len(sentence) <= max_length:
            current_segment += sentence
        else:
            if current_segment and len(current_segment) >= min_length:
                segments.append(current_segment.strip())
            current_segment = sentence
    
    if current_segment and len(current_segment) >= min_length:
        segments.append(current_segment.strip())
    elif current_segment and segments:
        segments[-1] += current_segment
    elif current_segment:
        segments.append(current_segment.strip())
    
    return segments


def main():
    """Main entry point for standalone PrimeSpeech node."""
    print("[PrimeSpeech] Starting standalone GPT-SoVITS TTS node...")
    
    # Get configuration
    voice_name = PrimeSpeechConfig.VOICE_NAME
    text_lang = PrimeSpeechConfig.TEXT_LANG
    speed_factor = PrimeSpeechConfig.SPEED_FACTOR
    device = PrimeSpeechConfig.DEVICE
    
    # Map voice names to simplified versions
    voice_map = {
        "Doubao": "assistant",
        "Luo Xiang": "narrator",
        "Yang Mi": "assistant",
        "default": "default"
    }
    
    mapped_voice = voice_map.get(voice_name, "default")
    
    # Initialize TTS engine
    try:
        tts_engine = GPTSoVITSEngine(voice_name=mapped_voice, device=device)
        print(f"[PrimeSpeech] Engine initialized with voice: {mapped_voice}")
    except Exception as e:
        print(f"[PrimeSpeech] Failed to initialize engine: {e}")
        sys.exit(1)
    
    # Initialize Dora node
    node = Node()
    print("[PrimeSpeech] Node initialized, waiting for text input...")
    
    # Send startup log
    send_log(node, "INFO", f"PrimeSpeech started with voice: {voice_name}")
    
    # Track statistics
    total_texts = 0
    total_audio_duration = 0
    session_start = time.time()
    
    # Main event loop
    for event in node:
        if event["type"] == "INPUT":
            input_id = event["id"]
            
            # Handle text input
            if input_id == "text":
                try:
                    # Get text
                    text = event["value"][0].as_py()
                    
                    if not text or not text.strip():
                        continue
                    
                    total_texts += 1
                    send_log(node, "INFO", f"Processing text #{total_texts}: {len(text)} chars")
                    
                    # Send status
                    node.send_output("status", pa.array(["tts_starting"]))
                    
                    # Segment text if needed
                    segments = segment_text_for_tts(text)
                    
                    # Process each segment
                    for seg_idx, segment in enumerate(segments):
                        start_time = time.time()
                        
                        # Synthesize speech
                        if PrimeSpeechConfig.RETURN_FRAGMENT:
                            # Streaming mode
                            chunk_count = 0
                            for audio_chunk, sample_rate in tts_engine.text_to_speech_streaming(
                                segment, text_lang, speed_factor
                            ):
                                # Send audio chunk
                                node.send_output(
                                    "audio",
                                    pa.array(audio_chunk.tobytes()),
                                    metadata={
                                        "sample_rate": sample_rate,
                                        "channels": 1,
                                        "format": "float32",
                                        "chunk_id": chunk_count,
                                        "segment_id": seg_idx,
                                        "is_last": False
                                    }
                                )
                                chunk_count += 1
                                
                                # Small delay between chunks
                                time.sleep(0.01)
                        else:
                            # Non-streaming mode
                            audio_data, sample_rate = tts_engine.text_to_speech(
                                segment, text_lang, speed_factor
                            )
                            
                            # Send complete audio
                            node.send_output(
                                "audio",
                                pa.array(audio_data.tobytes()),
                                metadata={
                                    "sample_rate": sample_rate,
                                    "channels": 1,
                                    "format": "float32",
                                    "segment_id": seg_idx,
                                    "is_last": seg_idx == len(segments) - 1
                                }
                            )
                        
                        # Calculate duration
                        synthesis_time = time.time() - start_time
                        audio_duration = len(audio_data) / sample_rate if 'audio_data' in locals() else 0
                        total_audio_duration += audio_duration
                        
                        send_log(node, "DEBUG", 
                                f"Segment {seg_idx + 1}/{len(segments)}: "
                                f"{synthesis_time:.2f}s synthesis, {audio_duration:.2f}s audio")
                        
                        # Send segment complete signal
                        node.send_output("segment_complete", pa.array([seg_idx]))
                    
                    # Send completion status
                    node.send_output("status", pa.array(["tts_complete"]))
                    send_log(node, "INFO", f"Completed text #{total_texts}")
                    
                except Exception as e:
                    send_log(node, "ERROR", f"TTS error: {str(e)}")
                    node.send_output("status", pa.array(["tts_error"]))
            
            # Handle control commands
            elif input_id == "control":
                try:
                    command = event["value"][0].as_py()
                    
                    if command == "reset":
                        # Reset engine
                        tts_engine = GPTSoVITSEngine(voice_name=mapped_voice, device=device)
                        send_log(node, "INFO", "Engine reset")
                        
                    elif command.startswith("voice:"):
                        # Change voice
                        new_voice = command.split(":", 1)[1]
                        mapped_new = voice_map.get(new_voice, "default")
                        tts_engine.set_voice(mapped_new)
                        send_log(node, "INFO", f"Voice changed to: {new_voice}")
                        
                    elif command == "stats":
                        # Send statistics
                        uptime = time.time() - session_start
                        stats = {
                            "texts_processed": total_texts,
                            "audio_duration": total_audio_duration,
                            "uptime": uptime,
                            "voice": voice_name
                        }
                        send_log(node, "INFO", f"Stats: {json.dumps(stats)}")
                        
                except Exception as e:
                    send_log(node, "ERROR", f"Control error: {str(e)}")
        
        elif event["type"] == "STOP":
            break
    
    # Cleanup
    send_log(node, "INFO", "PrimeSpeech shutting down")
    print(f"[PrimeSpeech] Processed {total_texts} texts, generated {total_audio_duration:.1f}s of audio")


if __name__ == "__main__":
    main()