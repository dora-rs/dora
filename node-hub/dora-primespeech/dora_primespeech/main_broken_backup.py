"""
Simplified Dora PrimeSpeech Node - Main entry point
High-quality text-to-speech using GPT-SoVITS technology.
"""

import os
import time
import json
import re
import numpy as np
import pyarrow as pa
from dora import Node
from pathlib import Path
from typing import Optional, List

from .config import PrimeSpeechConfig, VOICE_CONFIGS
from .model_manager import ModelManager
from .moyoyo_tts_wrapper_streaming_fix import StreamingMoYoYoTTSWrapper as MoYoYoTTSWrapper, MOYOYO_AVAILABLE


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
    node.send_output("log", pa.array([json.dumps(log_data)]))


def segment_text_for_tts(text: str, max_length: int = 100, min_length: int = 30) -> List[str]:
    """
    Segment long text into smaller chunks for faster TTS processing.
    Respects sentence boundaries and punctuation.
    
    Args:
        text: Input text to segment
        max_length: Maximum characters per segment
        min_length: Minimum characters per segment
    
    Returns:
        List of text segments
    """
    # Clean up text
    text = text.strip()
    text = re.sub(r'\s+', ' ', text)
    
    # If text is already short enough, return as is
    if len(text) <= max_length:
        return [text]
    
    # Split by sentence-ending punctuation (keeps punctuation)
    sentence_endings = r'([。！？.!?])'
    parts = re.split(sentence_endings, text)
    
    # Reconstruct complete sentences with their punctuation
    sentences = []
    for i in range(0, len(parts), 2):
        sentence = parts[i].strip()
        if i + 1 < len(parts):
            sentence += parts[i + 1]  # Add punctuation
        if sentence and not sentence.isspace():
            sentences.append(sentence)
    
    # Combine sentences into segments of appropriate length
    segments = []
    current_segment = ""
    
    for sentence in sentences:
        sentence = sentence.strip()
        if not sentence:
            continue
        
        # If current segment is empty, start with this sentence
        if not current_segment:
            current_segment = sentence
        
        # If adding this sentence keeps us under max, add it
        elif len(current_segment) + len(sentence) <= max_length:
            current_segment += " " + sentence  # Add space between sentences
        
        # If current segment is long enough, save it and start new
        elif len(current_segment) >= min_length:
            segments.append(current_segment)
            current_segment = sentence
        
        # Current segment too short but adding would exceed max
        else:
            # If the sentence alone is too long, split by commas
            if len(sentence) > max_length:
                # First, save current segment if it exists
                if current_segment and len(current_segment) >= min_length:
                    segments.append(current_segment)
                    current_segment = ""
                
                # Split long sentence by commas/semicolons
                clause_marks = r'([，,；;、])'
                clause_parts = re.split(clause_marks, sentence)
                
                # Reconstruct clauses
                for j in range(0, len(clause_parts), 2):
                    clause = clause_parts[j].strip()
                    if j + 1 < len(clause_parts):
                        clause += clause_parts[j + 1]
                    
                    if clause:
                        if not current_segment:
                            current_segment = clause
                        elif len(current_segment) + len(clause) <= max_length:
                            current_segment += clause
                        else:
                            if len(current_segment) >= min_length:
                                segments.append(current_segment)
                            current_segment = clause
            else:
                # Just add the sentence even if it makes segment a bit long
                current_segment += " " + sentence  # Add space between sentences
                if len(current_segment) >= min_length:
                    segments.append(current_segment)
                    current_segment = ""
    
    # Add final segment
    if current_segment:
        # If last segment is too short and we have previous segments, 
        # try to combine with the last one
        if len(current_segment) < min_length and segments:
            last_segment = segments[-1]
            if len(last_segment) + len(current_segment) <= max_length * 1.5:
                segments[-1] = last_segment + " " + current_segment  # Add space
            else:
                segments.append(current_segment)
        else:
            segments.append(current_segment)
    
    return segments


def main():
    """Main entry point for PrimeSpeech node"""
    
    node = Node()
    config = PrimeSpeechConfig()
    
    # Get voice configuration
    voice_name = config.VOICE_NAME
    if voice_name not in VOICE_CONFIGS:
        send_log(node, "ERROR", f"Unknown voice: {voice_name}. Available: {list(VOICE_CONFIGS.keys())}")
        voice_name = "Doubao"
    
    voice_config = VOICE_CONFIGS[voice_name]
    
    # Override with environment variables if provided
    if config.PROMPT_TEXT:
        voice_config["prompt_text"] = config.PROMPT_TEXT
    if config.TEXT_LANG != "auto":
        voice_config["text_lang"] = config.TEXT_LANG
    if config.PROMPT_LANG != "auto":
        voice_config["prompt_lang"] = config.PROMPT_LANG
    
    # Add inference parameters
    voice_config.update({
        "top_k": config.TOP_K,
        "top_p": config.TOP_P,
        "temperature": config.TEMPERATURE,
        "speed_factor": config.SPEED_FACTOR,
        "batch_size": config.BATCH_SIZE,
        "seed": config.SEED,
        "text_split_method": config.TEXT_SPLIT_METHOD,
        "split_bucket": config.SPLIT_BUCKET,
        "return_fragment": config.RETURN_FRAGMENT,
        "fragment_interval": config.FRAGMENT_INTERVAL,
        "use_gpu": config.USE_GPU,
        "device": config.DEVICE,
        "sample_rate": config.SAMPLE_RATE,
    })
    
    # Initialize model manager
    model_manager = ModelManager(config.get_models_dir())
    
    send_log(node, "INFO", "PrimeSpeech Node initialized")
    
    if MOYOYO_AVAILABLE:
        send_log(node, "INFO", "✓ MoYoYo TTS engine available")
    else:
        send_log(node, "WARNING", "⚠️  MoYoYo TTS not fully available")
    
    send_log(node, "INFO", f"Voice: {voice_name}")
    send_log(node, "INFO", f"Language: {voice_config.get('text_lang', 'auto')}")
    send_log(node, "INFO", f"Device: {config.DEVICE}")
    
    # Initialize TTS engine
    tts_engine: Optional[MoYoYoTTSWrapper] = None
    model_loaded = False
    
    # Statistics
    total_syntheses = 0
    total_duration = 0
    
    # Reset state management
    reset_counter = 0  # Increments on each reset
    current_reset_id = 0  # Current reset ID for tracking dirty segments
    pending_segments = []  # Queue of text segments waiting to be processed
    
    for event in node:
        if event["type"] == "INPUT":
            input_id = event["id"]
            
            if input_id == "text":
                # Get text to synthesize
                text = event["value"][0].as_py()
                metadata = event.get("metadata", {})
                
                session_id = metadata.get("session_id", "default")
                request_id = metadata.get("request_id", f"req_{total_syntheses}")
                segment_index = metadata.get("segment_index", -1)
                
                # Store the reset ID this segment belongs to
                segment_reset_id = current_reset_id
                
                # Check if we should segment the text internally
                enable_internal_segmentation = os.getenv("ENABLE_INTERNAL_SEGMENTATION", "true").lower() == "true"
                max_segment_length = int(os.getenv("TTS_MAX_SEGMENT_LENGTH", "100"))
                min_segment_length = int(os.getenv("TTS_MIN_SEGMENT_LENGTH", "30"))
                
                # Segment text if it's long
                if enable_internal_segmentation and len(text) > max_segment_length:
                    text_segments = segment_text_for_tts(text, max_segment_length, min_segment_length)
                    send_log(node, "INFO", f"Long text ({len(text)} chars) split into {len(text_segments)} segments for faster TTS")
                else:
                    text_segments = [text]
                
                send_log(node, "INFO", f"Processing segment {segment_index + 1} (len={len(text)}, parts={len(text_segments)}) [reset_id={segment_reset_id}]")
                
                # Load models if not loaded
                if not model_loaded:
                    send_log(node, "INFO", "Loading models for the first time...")
                    
                    if not model_manager.check_models_exist(voice_name, voice_config):
                        send_log(node, "INFO", f"Downloading models for {voice_name}...")
                        try:
                            model_paths = model_manager.get_voice_model_paths(voice_name, voice_config)
                            send_log(node, "INFO", "Models downloaded successfully")
                        except Exception as e:
                            send_log(node, "ERROR", f"Failed to download models: {e}")
                            continue
                    else:
                        repository = voice_config["repository"]
                        voice_dir = config.get_models_dir() / repository.replace("/", "_") / voice_name
                        model_paths = {
                            "gpt_model": voice_dir / voice_config["gpt_weights"],
                            "sovits_model": voice_dir / voice_config["sovits_weights"],
                            "reference_audio": voice_dir / voice_config["reference_audio"],
                        }
                    
                    # Initialize TTS engine
                    voice_map = {
                        "Doubao": "doubao",
                        "Luoxiang": "luoxiang",
                        "Yangmi": "yangmi",
                        "Yaoyao": "yaoyao",
                        "Xiaoyi": "xiaoyi"
                    }
                    moyoyo_voice = voice_map.get(voice_name, "doubao")
                    device = "cuda" if config.USE_GPU and config.DEVICE.startswith("cuda") else "cpu"
                    
                    enable_streaming = config.RETURN_FRAGMENT if hasattr(config, 'RETURN_FRAGMENT') else False
                    
                    tts_engine = MoYoYoTTSWrapper(
                        voice=moyoyo_voice, 
                        device=device,
                        enable_streaming=enable_streaming,
                        chunk_duration=0.3
                    )
                    
                    model_loaded = True
                    send_log(node, "INFO", "TTS engine ready")
                
                # Send status that TTS is starting
                node.send_output(
                    "status", 
                    pa.array(["tts_starting"]),
                    metadata={
                        "session_id": session_id,
                        "request_id": request_id,
                        "segment_index": segment_index,
                        "text_preview": text[:50]
                    }
                )
                
                # Synthesize speech for each text segment
                overall_start_time = time.time()
                total_audio_generated = 0
                
                try:
                    language = voice_config.get("text_lang", "zh")
                    speed = voice_config.get("speed_factor", 1.0)
                    
                    # Process each text segment
                    for part_idx, text_part in enumerate(text_segments):
                        # Check if we got a reset signal (between segments)
                        if current_reset_id != segment_reset_id:
                            send_log(node, "WARNING", f"Reset detected during synthesis, stopping segment processing")
                            break
                        
                        if len(text_segments) > 1:
                            send_log(node, "INFO", f"Synthesizing part {part_idx + 1}/{len(text_segments)} ({len(text_part)} chars)")
                        
                        start_time = time.time()
                        
                        if hasattr(tts_engine, 'enable_streaming') and tts_engine.enable_streaming:
                            # Streaming synthesis
                            send_log(node, "INFO", "Using streaming synthesis...")
                            fragment_num = 0
                            
                            for sample_rate, audio_fragment in tts_engine.synthesize_streaming(text_part, language=language, speed=speed):
                                fragment_num += 1
                                fragment_duration = len(audio_fragment) / sample_rate
                                total_audio_generated += fragment_duration
                                
                                node.send_output(
                                    "audio",
                                    pa.array([audio_fragment]),
                                    metadata={
                                        "session_id": session_id,
                                        "request_id": request_id,
                                        "segment_index": segment_index,
                                        "fragment_num": fragment_num,
                                        "sample_rate": sample_rate,
                                        "duration": fragment_duration,
                                        "is_streaming": True,
                                        "voice": voice_name,
                                        "language": language,
                                        "reset_id": segment_reset_id,  # Track which reset this belongs to
                                        "part_index": part_idx,
                                        "total_parts": len(text_segments)
                                    }
                                )
                            
                            synthesis_time = time.time() - start_time
                            send_log(node, "INFO", f"Part {part_idx + 1}: Streamed {fragment_num} fragments in {synthesis_time:.3f}s")
                            
                        else:
                            # Batch synthesis
                            sample_rate, audio_array = tts_engine.synthesize(text_part, language=language, speed=speed)
                            
                            synthesis_time = time.time() - start_time
                            audio_duration = len(audio_array) / sample_rate
                            total_audio_generated += audio_duration
                            
                            send_log(node, "INFO", f"Part {part_idx + 1}: Synthesized {audio_duration:.2f}s audio in {synthesis_time:.3f}s")
                            
                            # Send audio output
                            node.send_output(
                                "audio",
                                pa.array([audio_array]),
                                metadata={
                                    "session_id": session_id,
                                    "request_id": request_id,
                                    "segment_index": segment_index,
                                    "sample_rate": sample_rate,
                                    "duration": audio_duration,
                                    "synthesis_time": synthesis_time,
                                    "is_streaming": False,
                                    "voice": voice_name,
                                    "language": language,
                                    "reset_id": segment_reset_id,  # Track which reset this belongs to
                                    "part_index": part_idx,
                                    "total_parts": len(text_segments)
                                }
                            )
                    
                    # Update statistics
                    total_syntheses += 1
                    total_duration += total_audio_generated
                    
                    overall_synthesis_time = time.time() - overall_start_time
                    if len(text_segments) > 1:
                        send_log(node, "INFO", f"Completed all {len(text_segments)} parts: {total_audio_generated:.2f}s audio in {overall_synthesis_time:.3f}s")
                    
                    # Send segment completion signal
                    node.send_output(
                        "segment_complete",
                        pa.array(["completed"]),
                        metadata={
                            "session_id": session_id,
                            "request_id": request_id,
                            "segment_index": segment_index
                        }
                    )
                    send_log(node, "INFO", f"Sent segment_complete for segment {segment_index + 1}")
                    
                except Exception as e:
                    send_log(node, "ERROR", f"Synthesis error: {e}")
                    
                    # Send empty audio on error
                    node.send_output(
                        "audio",
                        pa.array([np.array([], dtype=np.float32)]),
                        metadata={
                            "session_id": session_id,
                            "request_id": request_id,
                            "error": str(e),
                            "sample_rate": config.SAMPLE_RATE
                        }
                    )
            
            elif input_id == "control":
                # Handle control commands
                command = event["value"][0].as_py()
                
                if command == "reset":
                    reset_counter += 1
                    old_reset_id = current_reset_id
                    current_reset_id = reset_counter
                    
                    send_log(node, "INFO", f"[PrimeSpeech] RESET received - incrementing reset_id from {old_reset_id} to {current_reset_id}")
                    
                    # Clear any pending segments (though we process immediately now)
                    pending_segments.clear()
                    
                    # Send reset acknowledgment with new reset_id
                    node.send_output(
                        "status",
                        pa.array(["reset_acknowledged"]),
                        metadata={
                            "reset_id": current_reset_id,
                            "old_reset_id": old_reset_id,
                            "timestamp": time.time()
                        }
                    )
                    
                    send_log(node, "INFO", f"[PrimeSpeech] Reset acknowledged - new reset_id={current_reset_id}")
                
                elif command == "stats":
                    send_log(node, "INFO", f"Total syntheses: {total_syntheses}")
                    send_log(node, "INFO", f"Total audio duration: {total_duration:.1f}s")
        
        elif event["type"] == "STOP":
            break
    
    send_log(node, "INFO", "PrimeSpeech node stopped")


if __name__ == "__main__":
    main()