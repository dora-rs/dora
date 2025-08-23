#!/usr/bin/env python3
"""
Test script to verify AEC node output
"""

import time
from dora import Node

def main():
    node = Node("test-receiver")
    
    print("Test Receiver: Waiting for audio from AEC...")
    
    event_count = 0
    audio_count = 0
    
    for event in node:
        event_count += 1
        
        if event["type"] == "INPUT":
            input_id = event.get("id", "")
            
            if input_id == "audio":
                audio_count += 1
                audio_data = event["value"].to_numpy()
                print(f"Received audio #{audio_count}: {len(audio_data)} samples, type: {audio_data.dtype}")
                
                # Show first few samples
                if len(audio_data) > 0:
                    print(f"  First 10 samples: {audio_data[:10]}")
                    print(f"  Max amplitude: {abs(audio_data).max():.4f}")
            
            elif input_id == "log":
                log_msg = event["value"][0].as_py()
                print(f"Log: {log_msg}")
        
        # Exit after some events for testing
        if event_count > 100:
            print(f"Test complete: {event_count} events, {audio_count} audio packets")
            break

if __name__ == "__main__":
    main()