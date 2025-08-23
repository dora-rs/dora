#!/usr/bin/env python3
"""
Minimal logger to debug AEC + Speech Monitor connection
"""

import sys
import time
import numpy as np
from dora import Node

def main():
    node = Node("logger")
    
    print("\n" + "="*60)
    print("MINIMAL TEST: AEC -> Speech Monitor")
    print("="*60)
    print("Waiting for events...\n")
    sys.stdout.flush()
    
    event_count = 0
    audio_segment_count = 0
    aec_log_count = 0
    monitor_log_count = 0
    
    start_time = time.time()
    
    for event in node:
        event_count += 1
        
        if event["type"] == "INPUT":
            input_id = event.get("id", "")
            
            if input_id == "aec_log":
                aec_log_count += 1
                try:
                    log_msg = event["value"][0].as_py()
                    print(f"[AEC LOG #{aec_log_count}] {log_msg}")
                except:
                    print(f"[AEC LOG #{aec_log_count}] (unable to parse)")
                    
            elif input_id == "monitor_log":
                monitor_log_count += 1
                try:
                    log_msg = event["value"][0].as_py()
                    print(f"[MONITOR LOG #{monitor_log_count}] {log_msg}")
                except:
                    print(f"[MONITOR LOG #{monitor_log_count}] (unable to parse)")
            
            elif input_id == "asr_log":
                monitor_log_count += 1
                try:
                    log_msg = event["value"][0].as_py()
                    print(f"[asr LOG #{monitor_log_count}] {log_msg}")
                except:
                    print(f"[asr LOG #{monitor_log_count}] (unable to parse)")


            elif input_id == "audio_segment":
                audio_segment_count += 1
                try:
                    audio_data = event["value"].to_numpy()
                    duration_ms = len(audio_data) / 16  # 16 samples per ms at 16kHz
                    max_amp = np.abs(audio_data).max()
                    print(f"\n*** AUDIO SEGMENT #{audio_segment_count} ***")
                    print(f"    Size: {len(audio_data)} samples")
                    print(f"    Duration: {duration_ms:.1f} ms")
                    print(f"    Max amplitude: {max_amp:.4f}")
                    print(f"    Time since start: {time.time() - start_time:.1f}s\n")
                except Exception as e:
                    print(f"[AUDIO SEGMENT #{audio_segment_count}] Error: {e}")
        
        # Print status every 100 events
        if event_count % 100 == 0:
            elapsed = time.time() - start_time
            print(f"\n--- STATUS after {elapsed:.1f}s ---")
            print(f"Total events: {event_count}")
            print(f"AEC logs: {aec_log_count}")
            print(f"Monitor logs: {monitor_log_count}")
            print(f"Audio segments: {audio_segment_count}")
            print("-" * 30 + "\n")
        
        sys.stdout.flush()
        
        # Stop after 30 seconds for testing
        if time.time() - start_time > 30:
            print(f"\n\nTest complete after 30 seconds")
            print(f"Final stats:")
            print(f"  Total events: {event_count}")
            print(f"  AEC logs: {aec_log_count}")
            print(f"  Monitor logs: {monitor_log_count}")
            print(f"  Audio segments detected: {audio_segment_count}")
            break

if __name__ == "__main__":
    main()