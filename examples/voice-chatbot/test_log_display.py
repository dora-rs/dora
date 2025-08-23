#!/usr/bin/env python3
"""
Test script for the log display node.
Generates sample log messages to test formatting and display.
"""

import json
import time
import random
from dora import Node
import pyarrow as pa


def generate_sample_logs():
    """Generate sample log messages for testing."""
    
    nodes = ["speech-monitor", "asr", "chat-controller", "qwen3-llm", "text-segmenter", "primespeech"]
    levels = ["DEBUG", "INFO", "WARNING", "ERROR"]
    
    sample_messages = {
        "speech-monitor": [
            "Speech Monitor initialized",
            "Speech STARTED (segment #1)",
            "Trailing silence...",
            "Speech SEGMENT sent: 2.34s",
            "Question ENDED (silence: 3045ms)",
            "Speech Monitor PAUSED",
            "Speech Monitor RESUMED"
        ],
        "asr": [
            "ASR Engine initialized: zh",
            "Processing audio segment (2.3s)",
            "Transcription: 今天天气怎么样",
            "Language detected: zh (confidence: 0.95)",
            "Processing time: 0.234s"
        ],
        "chat-controller": [
            "CHAT CONTROLLER STARTED",
            "Ready for conversation...",
            "New speech segment started",
            "Accumulated transcription: 今天天气",
            "Question complete (silence: 3000ms): 今天天气怎么样",
            "Sending question to LLM",
            "Answer playback started",
            "Answer playback complete (5.2s, buffer: 3.1%)"
        ],
        "qwen3-llm": [
            "MLX Model loaded: Qwen3-8B-MLX-4bit",
            "Processing question: 今天天气怎么样",
            "Generated response in 1.23s",
            "Token count: 45"
        ],
        "text-segmenter": [
            "Received text for segmentation",
            "Segment 1: 今天的天气很好。",
            "Segment 2: 温度适中，阳光明媚。",
            "All segments sent"
        ],
        "primespeech": [
            "TTS initialized with voice: Zhiyu",
            "Processing segment: 今天的天气很好。",
            "Audio generated: 2.1s duration",
            "Segment complete signal sent"
        ]
    }
    
    logs = []
    current_time = time.time()
    
    # Generate a sequence of logs
    for i in range(30):
        node = random.choice(nodes)
        level = random.choices(levels, weights=[20, 60, 15, 5])[0]  # More INFO logs
        messages = sample_messages[node]
        message = random.choice(messages)
        
        log_data = {
            "node": node,
            "level": level,
            "message": f"[{level}] {message}",
            "timestamp": current_time + i * 0.5  # Spread over 15 seconds
        }
        logs.append(log_data)
    
    return logs


def main():
    """Send test logs to the log display node."""
    
    node = Node("test-log-sender")
    
    print("Starting log test sender...")
    print("Generating sample logs...")
    
    sample_logs = generate_sample_logs()
    
    print(f"Sending {len(sample_logs)} test logs over 15 seconds...")
    
    start_time = time.time()
    
    for i, log_data in enumerate(sample_logs):
        # Wait until it's time to send this log
        target_time = log_data["timestamp"]
        current_time = time.time()
        wait_time = target_time - current_time
        
        if wait_time > 0:
            time.sleep(wait_time)
        
        # Send the log
        for output_name in ["speech_monitor_log", "asr_log", "chat_controller_log", 
                           "llm_log", "text_segmenter_log", "primespeech_log"]:
            if log_data["node"].replace("-", "_") in output_name:
                node.send_output(output_name, pa.array([json.dumps(log_data)]))
                print(f"Sent log {i+1}/{len(sample_logs)}: {log_data['node']} - {log_data['message'][:50]}...")
                break
    
    print("\nAll test logs sent!")
    print("Check the log display node to see formatted output.")
    

if __name__ == "__main__":
    main()