#!/usr/bin/env python3
"""
Monitor and display audio metrics from mac-aec node
"""

import json
import time
import pyarrow as pa
from dora import Node


def format_metrics(metrics):
    """Format metrics for display"""
    quality = metrics.get('audio_quality', 'unknown')
    
    # Color codes for terminal
    colors = {
        'good': '\033[92m',      # Green
        'acceptable': '\033[93m', # Yellow
        'poor_clipping': '\033[91m', # Red
        'too_quiet': '\033[94m',  # Blue
        'too_loud': '\033[91m',   # Red
        'unknown': '\033[90m',    # Gray
    }
    reset = '\033[0m'
    
    quality_color = colors.get(quality, '')
    
    output = f"""
╔══════════════════════════════════════════════╗
║            AUDIO METRICS MONITOR             ║
╠══════════════════════════════════════════════╣
║ Quality: {quality_color}{quality:12s}{reset}                  ║
║ RMS:     {metrics.get('current_rms', 0):.4f} (avg: {metrics.get('rms_mean', 0):.4f})      ║
║ Peak:    {metrics.get('current_peak', 0):.4f} (max: {metrics.get('peak_max', 0):.4f})      ║
║ VAD:     {metrics.get('vad_positive_rate', 0)*100:.1f}% active                    ║
║ Chunks:  {metrics.get('total_chunks', 0):6d}                        ║
║ Clipping:{metrics.get('clipping_rate', 0)*100:5.1f}%                        ║
║ Silence: {metrics.get('silence_rate', 0)*100:5.1f}%                        ║
╚══════════════════════════════════════════════╝
    """
    return output.strip()


def main():
    node = Node()
    print("Starting metrics monitor...")
    
    last_update = time.time()
    
    while True:
        event = node.next(timeout=0.1)
        
        if event and event["type"] == "INPUT":
            if event["id"] == "metrics":
                try:
                    metrics_json = event["value"][0].as_py()
                    metrics = json.loads(metrics_json)
                    
                    # Clear screen and display
                    print("\033[2J\033[H")  # Clear screen
                    print(format_metrics(metrics))
                    
                    # Send report
                    report = f"Audio quality: {metrics.get('audio_quality', 'unknown')}"
                    node.send_output("report", pa.array([report]))
                    
                except Exception as e:
                    print(f"Error processing metrics: {e}")
            
            elif event["id"] == "status":
                try:
                    status_json = event["value"][0].as_py()
                    status = json.loads(status_json)
                    print(f"\nStatus: {status.get('status')} - Frame #{status.get('frame_count', 0)}")
                except:
                    pass


if __name__ == "__main__":
    main()