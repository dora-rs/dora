"""
Simplified Dora PrimeSpeech Node - Main entry point
High-quality text-to-speech using GPT-SoVITS technology.
"""

import time
import json
import numpy as np
import pyarrow as pa
from dora import Node
from pathlib import Path
from typing import Optional

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
                
                send_log(node, "INFO", f"Processing segment {segment_index + 1} (len={len(text)})")
                
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
                
                # Synthesize speech
                start_time = time.time()
                
                try:
                    language = voice_config.get("text_lang", "zh")
                    speed = voice_config.get("speed_factor", 1.0)
                    
                    if hasattr(tts_engine, 'enable_streaming') and tts_engine.enable_streaming:
                        # Streaming synthesis
                        send_log(node, "INFO", "Using streaming synthesis...")
                        fragment_num = 0
                        total_audio_duration = 0
                        
                        for sample_rate, audio_fragment in tts_engine.synthesize_streaming(text, language=language, speed=speed):
                            fragment_num += 1
                            fragment_duration = len(audio_fragment) / sample_rate
                            total_audio_duration += fragment_duration
                            
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
                                    "language": language
                                }
                            )
                        
                        synthesis_time = time.time() - start_time
                        send_log(node, "INFO", f"Streamed {fragment_num} fragments, {total_audio_duration:.2f}s audio in {synthesis_time:.3f}s")
                        
                    else:
                        # Batch synthesis
                        sample_rate, audio_array = tts_engine.synthesize(text, language=language, speed=speed)
                        
                        synthesis_time = time.time() - start_time
                        audio_duration = len(audio_array) / sample_rate
                        
                        total_syntheses += 1
                        total_duration += audio_duration
                        
                        send_log(node, "INFO", f"Synthesized: {audio_duration:.2f}s audio in {synthesis_time:.3f}s")
                        
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
                                "language": language
                            }
                        )
                    
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
                    send_log(node, "INFO", "[PrimeSpeech] RESET received")
                    # Note: Can't actually stop ongoing synthesis, but it's OK
                    # because we only process one segment at a time now
                    send_log(node, "INFO", "[PrimeSpeech] Reset acknowledged")
                
                elif command == "stats":
                    send_log(node, "INFO", f"Total syntheses: {total_syntheses}")
                    send_log(node, "INFO", f"Total audio duration: {total_duration:.1f}s")
        
        elif event["type"] == "STOP":
            break
    
    send_log(node, "INFO", "PrimeSpeech node stopped")


if __name__ == "__main__":
    main()