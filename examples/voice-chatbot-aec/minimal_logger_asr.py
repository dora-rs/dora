#!/usr/bin/env python3
"""
Minimal logger with ASR support to test AEC -> Speech Monitor -> ASR pipeline
"""

import sys
import time
import json
import numpy as np
from dora import Node

def main():
    node = Node("logger")
    
    print("\n" + "="*70)
    print("MINIMAL TEST: AEC -> Speech Monitor -> ASR")
    print("="*70)
    print("\nSpeak clearly to test the pipeline:")
    print("1. AEC captures your voice with echo cancellation")
    print("2. Speech Monitor detects speech segments")
    print("3. ASR transcribes the speech to text")
    print("\nWaiting for events...\n")
    sys.stdout.flush()
    
    event_count = 0
    audio_segment_count = 0
    transcription_count = 0
    
    start_time = time.time()
    last_status_time = start_time
    
    for event in node:
        event_count += 1
        
        if event["type"] == "INPUT":
            input_id = event.get("id", "")
            
            # Handle transcribed text from ASR
            if input_id == "asr_text":
                transcription_count += 1
                try:
                    text = event["value"][0].as_py()
                    elapsed = time.time() - start_time
                    
                    print("\n" + "="*70)
                    print(f"🎤 TRANSCRIPTION #{transcription_count} at {elapsed:.1f}s")
                    print(f"📝 Text: {text}")
                    print("="*70 + "\n")
                    
                except Exception as e:
                    print(f"[ASR TEXT] Error parsing: {e}")
            
            # Handle speech started
            elif input_id == "speech_started":
                try:
                    timestamp = event["value"][0].as_py()
                    elapsed = time.time() - start_time
                    print(f"\n🔴 Speech STARTED at {elapsed:.1f}s")
                except:
                    pass
            
            # Handle speech ended
            elif input_id == "speech_ended":
                try:
                    timestamp = event["value"][0].as_py()
                    elapsed = time.time() - start_time
                    print(f"⏹️  Speech ENDED at {elapsed:.1f}s")
                except:
                    pass
            
            # Handle audio segments
            elif input_id == "audio_segment":
                audio_segment_count += 1
                try:
                    audio_data = event["value"].to_numpy()
                    duration_ms = len(audio_data) / 16  # 16 samples per ms at 16kHz
                    max_amp = np.abs(audio_data).max()
                    print(f"   Audio segment: {len(audio_data)} samples ({duration_ms:.0f}ms), max amp: {max_amp:.3f}")
                except Exception as e:
                    print(f"   Audio segment error: {e}")
            
            # Handle logs
            elif input_id.endswith("_log"):
                try:
                    log_value = event["value"][0].as_py()
                    
                    # Parse JSON logs
                    if isinstance(log_value, str) and log_value.startswith("{"):
                        try:
                            log_data = json.loads(log_value)
                            node_name = log_data.get("node", "unknown")
                            level = log_data.get("level", "INFO")
                            message = log_data.get("message", "")
                            
                            # Filter out debug messages for cleaner output
                            if level != "DEBUG":
                                # Special formatting for important messages
                                if "Processing segment" in message:
                                    print(f"   [ASR] {message}")
                                elif "Speech STARTED" in message or "Speech ENDED" in message:
                                    # Skip, we handle these separately
                                    pass
                                elif "initialized" in message or "ready" in message:
                                    print(f"✅ {message}")
                                elif level == "ERROR":
                                    print(f"❌ {message}")
                                elif level == "WARNING":
                                    print(f"⚠️  {message}")
                        except:
                            # Not JSON, print as-is if it's important
                            if "[INFO]" in log_value or "[ERROR]" in log_value:
                                print(f"   {log_value}")
                    
                except Exception as e:
                    pass  # Silently ignore log parsing errors
        
        # Print periodic status
        current_time = time.time()
        if current_time - last_status_time > 10.0:
            elapsed = current_time - start_time
            print(f"\n--- Status at {elapsed:.1f}s ---")
            print(f"Events processed: {event_count}")
            print(f"Audio segments: {audio_segment_count}")
            print(f"Transcriptions: {transcription_count}")
            print("-" * 30)
            last_status_time = current_time
        
        sys.stdout.flush()
        
        # Stop after 60 seconds for testing
        if time.time() - start_time > 60:
            print(f"\n\n" + "="*70)
            print("Test complete after 60 seconds")
            print(f"Final statistics:")
            print(f"  Total events: {event_count}")
            print(f"  Audio segments detected: {audio_segment_count}")
            print(f"  Transcriptions completed: {transcription_count}")
            
            if transcription_count == 0:
                print("\n⚠️  No transcriptions detected!")
                print("  Make sure to speak clearly into the microphone")
            else:
                print(f"\n✅ Successfully transcribed {transcription_count} speech segments!")
            
            print("="*70)
            break

if __name__ == "__main__":
    main()