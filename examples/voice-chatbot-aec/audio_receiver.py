#!/usr/bin/env python3
"""
Simple audio receiver to debug AEC output
"""

import sys
import numpy as np
from dora import Node, DoraStatus

def main():
    node = Node("audio-player")
    
    print("[AUDIO RECEIVER] Started, waiting for audio...", flush=True)
    sys.stdout.flush()
    
    event_count = 0
    audio_count = 0
    
    for event in node:
        event_count += 1
        
        try:
            if event["type"] == "INPUT":
                input_id = event.get("id", "unknown")
                
                if input_id == "audio":
                    audio_count += 1
                    value = event.get("value", None)
                    
                    if value is not None:
                        try:
                            if hasattr(value, 'to_numpy'):
                                data = value.to_numpy()
                                max_val = np.abs(data).max() if len(data) > 0 else 0
                                print(f"[AUDIO RECEIVER] Audio #{audio_count}: {len(data)} samples, dtype={data.dtype}, max_amp={max_val:.4f}", flush=True)
                                
                                # Print first few samples
                                if len(data) > 0 and audio_count <= 5:
                                    print(f"  First 10 samples: {data[:10]}", flush=True)
                            else:
                                print(f"[AUDIO RECEIVER] Audio #{audio_count}: Non-numpy audio data", flush=True)
                        except Exception as e:
                            print(f"[AUDIO RECEIVER] Audio #{audio_count}: Error processing: {e}", flush=True)
                    else:
                        print(f"[AUDIO RECEIVER] Audio #{audio_count}: No value", flush=True)
                        
                elif input_id == "log":
                    try:
                        value = event.get("value", None)
                        if value and hasattr(value, '__getitem__'):
                            log_msg = value[0].as_py() if hasattr(value[0], 'as_py') else str(value[0])
                            print(f"[AUDIO LOG] {log_msg}", flush=True)
                    except Exception as e:
                        print(f"[AUDIO RECEIVER] Log error: {e}", flush=True)
                        
            elif event["type"] == "STOP":
                print(f"[AUDIO RECEIVER] Received STOP event", flush=True)
                break
                
        except Exception as e:
            print(f"[AUDIO RECEIVER] Error processing event #{event_count}: {e}", flush=True)
        
        sys.stdout.flush()
        
        # Stop after receiving some audio to avoid running forever
        if audio_count >= 20:
            print(f"[AUDIO RECEIVER] Received {audio_count} audio packets, stopping...", flush=True)
            break
    
    print(f"[AUDIO RECEIVER] Exiting. Total events: {event_count}, Audio packets: {audio_count}", flush=True)
    return DoraStatus.STOP

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"[AUDIO RECEIVER] Fatal error: {e}", flush=True)
        sys.exit(1)