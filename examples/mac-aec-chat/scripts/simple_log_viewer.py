#!/usr/bin/env python3
"""
Simple log viewer for mac-aec and speech monitor output
"""

import json
import time
from dora import Node

def main():
    print("=" * 60)
    print("SPEECH MONITOR LOG VIEWER")
    print("=" * 60)
    print("Listening for events...")
    print("Speak into your microphone to test voice detection!")
    print()
    
    node = Node("log-viewer")
    
    last_speech_start = None
    last_speech_end = None
    is_speaking = False
    speech_count = 0
    event_count = 0
    
    while True:
        event = node.next()
        
        if event and event["type"] == "INPUT":
            input_id = event["id"]
            event_count += 1
            
            # Print first 10 events to debug what we're receiving
            if event_count <= 10:
                print(f"[DEBUG] Event #{event_count}: input_id='{input_id}'")
            
            if input_id == "speech_log":
                # Log from speech monitor - filter out debug chunk messages
                log_data = event["value"][0].as_py()
                # Skip debug audio chunk messages but show other logs
                if "Received audio chunk" not in log_data:
                    try:
                        log_json = json.loads(log_data)
                        msg = log_json.get("message", log_data)
                        if "Speech started" in msg or "Speech ended" in msg or "is_speaking" in msg:
                            print(f"[IMPORTANT] {msg}")
                        elif event_count <= 100:  # Show first 100 logs for debugging
                            print(f"[LOG] {msg}")
                    except:
                        if event_count <= 100:
                            print(f"[LOG] {log_data}")
                
            elif input_id == "mac_status":
                # Status from mac-aec
                status_json = event["value"][0].as_py()
                try:
                    status = json.loads(status_json)
                    if status.get("status") == "current":
                        frames = status.get("frame_count", 0)
                        if event_count % 100 == 0:  # Only print occasionally
                            print(f"[MAC-AEC STATUS] Frames: {frames}")
                except:
                    pass
                    
            elif input_id == "speech_started":
                # Speech started event
                speech_count += 1
                last_speech_start = time.time()
                is_speaking = True
                print(f"\n{'='*60}")
                print(f"🎤 SPEECH STARTED (#{speech_count})")
                print(f"{'='*60}\n")
                
            elif input_id == "speech_ended":
                # Speech ended event
                last_speech_end = time.time()
                is_speaking = False
                if last_speech_start:
                    duration = last_speech_end - last_speech_start
                    print(f"\n{'='*60}")
                    print(f"🔇 SPEECH ENDED (duration: {duration:.2f}s)")
                    print(f"{'='*60}\n")
                else:
                    print(f"\n🔇 SPEECH ENDED\n")
                    
            elif input_id == "is_speaking":
                # Speaking status update
                speaking = event["value"][0].as_py()
                if speaking != is_speaking:
                    is_speaking = speaking
                    status = "SPEAKING" if speaking else "SILENT"
                    print(f"\n[VOICE STATUS CHANGED] {status}\n")
                    
            elif input_id == "speech_probability":
                # Show speech probability occasionally
                prob = event["value"][0].as_py()
                if event_count % 50 == 0:  # Every 50th event
                    print(f"[Speech Probability] {prob:.3f}")
                    
            elif input_id == "audio_segment":
                # Audio segment received
                print(f"[AUDIO SEGMENT] Received audio segment")
                
            else:
                # Unknown input
                if event_count <= 20:
                    print(f"[UNKNOWN INPUT] {input_id}")
        
        # Print status every 100 events
        if event_count % 100 == 0:
            print(f"[STATUS] Events received: {event_count}, Speech detections: {speech_count}")


if __name__ == "__main__":
    main()
