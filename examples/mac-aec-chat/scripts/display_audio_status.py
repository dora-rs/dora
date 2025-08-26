#!/usr/bin/env python3
"""
Simple display node to test mac-aec output
"""

import time
import json
import numpy as np
import pyarrow as pa
from dora import Node


def main():
    node = Node()
    
    print("=" * 60)
    print("MAC-AEC TEST DISPLAY")
    print("=" * 60)
    print("Waiting for audio data from mac-aec node...")
    print()
    
    audio_chunks = 0
    vad_detections = 0
    last_status = None
    last_metrics = None
    start_time = time.time()
    
    while True:
        event = node.next(timeout=0.1)
        
        if event and event["type"] == "INPUT":
            input_id = event["id"]
            
            if input_id == "audio":
                # Audio data received
                audio_data = event["value"].to_numpy()
                audio_chunks += 1
                
                # Calculate audio stats
                rms = np.sqrt(np.mean(audio_data ** 2))
                peak = np.max(np.abs(audio_data))
                
                elapsed = time.time() - start_time
                print(f"[{elapsed:6.1f}s] Audio chunk #{audio_chunks}: RMS={rms:.4f}, Peak={peak:.4f}, Samples={len(audio_data)}")
                
            elif input_id == "vad":
                # VAD status
                vad_active = event["value"][0].as_py()
                if vad_active:
                    vad_detections += 1
                    print(f"  >>> VOICE DETECTED! (Total detections: {vad_detections})")
                
            elif input_id == "status":
                # Status update
                try:
                    status_json = event["value"][0].as_py()
                    status = json.loads(status_json)
                    last_status = status
                    print(f"  Status: {status.get('status', 'unknown')}")
                except:
                    pass
                
            elif input_id == "metrics":
                # Metrics update
                try:
                    metrics_json = event["value"][0].as_py()
                    metrics = json.loads(metrics_json)
                    last_metrics = metrics
                    
                    # Display metrics
                    print(f"  Metrics: Quality={metrics.get('audio_quality', 'unknown')}, "
                          f"RMS={metrics.get('current_rms', 0):.4f}, "
                          f"VAD={metrics.get('vad_positive_rate', 0)*100:.1f}%")
                except:
                    pass
            
            # Send display status
            node.send_output("display", pa.array([f"Chunks: {audio_chunks}, VAD: {vad_detections}"]))
            
        elif event and event["type"] == "STOP":
            break
    
    # Final summary
    elapsed = time.time() - start_time
    print()
    print("=" * 60)
    print("TEST SUMMARY")
    print("=" * 60)
    print(f"Duration: {elapsed:.1f}s")
    print(f"Audio chunks received: {audio_chunks}")
    print(f"Voice detections: {vad_detections}")
    if last_metrics:
        print(f"Final audio quality: {last_metrics.get('audio_quality', 'unknown')}")
    print("=" * 60)


if __name__ == "__main__":
    main()