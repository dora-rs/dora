"""
Dora ASR Node - Main entry point
Multi-engine ASR with task management and interruption handling.
"""

import time
import json
import numpy as np
import pyarrow as pa
from dora import Node

from .config import ASRConfig
from .manager import ASRManager
from .utils import (
    calculate_audio_stats,
    normalize_transcription,
    split_audio_for_long_transcription,
    merge_transcription_chunks
)


def send_log(node, level, message):
    """Send log message through log output channel.
    
    Args:
        node: Dora node instance
        level: Log level (DEBUG, INFO, WARNING, ERROR)
        message: Log message
    """
    # Define log level hierarchy
    LOG_LEVELS = {
        "DEBUG": 10,
        "INFO": 20,
        "WARNING": 30,
        "ERROR": 40
    }
    
    # Get configured log level
    from .config import ASRConfig
    config_level = ASRConfig.LOG_LEVEL
    
    # Check if message should be logged
    if LOG_LEVELS.get(level, 0) < LOG_LEVELS.get(config_level, 20):
        return  # Skip messages below configured level
    
    # Format message with level prefix
    formatted_message = f"[{level}] {message}"
    
    log_data = {
        "node": "asr",
        "level": level,
        "message": formatted_message,
        "timestamp": time.time()
    }
    node.send_output("log", pa.array([json.dumps(log_data)]))


def main():
    """Main entry point for ASR node"""
    
    # Initialize
    node = Node()
    config = ASRConfig()
    manager = ASRManager(node)  # Pass node for logging
    
    # Send initialization logs
    send_log(node, "INFO", "ASR Node initialized")
    send_log(node, "INFO", f"Engine: {config.ASR_ENGINE}")
    send_log(node, "INFO", f"Language: {config.LANGUAGE}")
    send_log(node, "INFO", f"Log level: {config.LOG_LEVEL}")
    send_log(node, "DEBUG", f"Punctuation: {config.ENABLE_PUNCTUATION}")
    send_log(node, "DEBUG", f"Models directory: {config.get_models_dir()}")
    
    # Statistics
    total_segments = 0
    total_duration = 0
    
    for event in node:
        if event["type"] == "INPUT":
            input_id = event["id"]
            
            if input_id == "audio":
                # Get audio segment
                audio_array = event["value"].to_numpy()
                metadata = event.get("metadata", {})
                
                # Extract metadata from speech monitor
                task_id = metadata.get("task_id", "unknown")
                segment_num = metadata.get("segment", 0)
                sample_rate = metadata.get("sample_rate", config.SAMPLE_RATE)
                
                # Calculate audio statistics
                audio_stats = calculate_audio_stats(audio_array)
                duration = audio_stats['duration']
                
                send_log(node, "INFO", f"Processing segment #{segment_num}")
                send_log(node, "DEBUG", f"   Duration: {duration:.2f}s")
                send_log(node, "DEBUG", f"   Task ID: {task_id[:8]}...")
                
                start_time = time.time()
                
                try:
                    # Check if audio is too long and needs splitting
                    if duration > config.MAX_AUDIO_DURATION:
                        send_log(node, "WARNING", f"Audio too long ({duration:.1f}s), splitting...")
                        
                        # Split into chunks
                        chunks = split_audio_for_long_transcription(
                            audio_array,
                            sample_rate=sample_rate,
                            chunk_duration=config.MAX_AUDIO_DURATION,
                            overlap_duration=1.0
                        )
                        
                        # Transcribe each chunk
                        transcribed_chunks = []
                        for i, chunk_data in enumerate(chunks):
                            send_log(node, "DEBUG", f"Processing chunk {i+1}/{len(chunks)}...")
                            result = manager.transcribe(
                                chunk_data['audio'],
                                language=config.LANGUAGE
                            )
                            transcribed_chunks.append({
                                'text': result['text'],
                                'start_time': chunk_data['start_time'],
                                'end_time': chunk_data['end_time']
                            })
                        
                        # Merge results
                        full_text = merge_transcription_chunks(transcribed_chunks)
                        detected_language = config.LANGUAGE
                        
                    else:
                        # Normal transcription
                        result = manager.transcribe(
                            audio_array,
                            language=config.LANGUAGE
                        )
                        full_text = result['text']
                        detected_language = result['language']
                    
                    # Normalize text
                    full_text = normalize_transcription(full_text, detected_language)
                    
                    processing_time = time.time() - start_time
                    
                    # Update statistics
                    total_segments += 1
                    total_duration += duration
                    
                    # Skip empty transcriptions
                    if not full_text.strip():
                        send_log(node, "WARNING", "Empty transcription")
                        continue
                    
                    send_log(node, "INFO", f"Transcribed: {full_text[:100]}...")
                    send_log(node, "INFO", f"Language: {detected_language}")
                    send_log(node, "DEBUG", f"Processing time: {processing_time:.3f}s")
                    send_log(node, "DEBUG", f"Speed: {duration/processing_time:.1f}x realtime")
                    
                    # Send transcription output
                    node.send_output(
                        "transcription",
                        pa.array([full_text]),
                        metadata={
                            "task_id": task_id,
                            "segment": segment_num,
                            "duration": duration,
                            "timestamp": time.time()
                        }
                    )
                    
                    # Send language detection if enabled
                    if config.ENABLE_LANGUAGE_DETECTION:
                        node.send_output(
                            "language_detected",
                            pa.array([detected_language]),
                            metadata={"task_id": task_id}
                        )
                    
                    # Send processing time
                    node.send_output(
                        "processing_time",
                        pa.array([processing_time]),
                        metadata={
                            "task_id": task_id,
                            "audio_duration": duration,
                            "speed_ratio": duration / processing_time
                        }
                    )
                    
                    # Send confidence if available
                    if config.ENABLE_CONFIDENCE_SCORE and result.get('confidence'):
                        node.send_output(
                            "confidence",
                            pa.array([result['confidence']]),
                            metadata={"task_id": task_id}
                        )
                    
                except Exception as e:
                    send_log(node, "ERROR", f"Transcription error: {e}")
                    
                    # Send empty transcription on error
                    node.send_output(
                        "transcription",
                        pa.array([""]),
                        metadata={
                            "task_id": task_id,
                            "error": str(e)
                        }
                    )
            
            elif input_id == "control":
                # Handle control commands
                command = event["value"][0].as_py()
                
                if command == "stats":
                    # Report statistics
                    send_log(node, "INFO", "ASR Statistics:")
                    send_log(node, "INFO", f"Total segments: {total_segments}")
                    send_log(node, "INFO", f"Total duration: {total_duration:.1f}s")
                    if total_segments > 0:
                        send_log(node, "INFO", f"Average duration: {total_duration/total_segments:.1f}s")
                
                elif command == "cleanup":
                    # Cleanup resources
                    send_log(node, "INFO", "Cleaning up ASR engines...")
                    manager.cleanup()
                    send_log(node, "INFO", "Cleanup complete")


if __name__ == "__main__":
    main()