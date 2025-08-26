#!/usr/bin/env python3
"""
Simple passthrough segmenter - just forwards text immediately
"""

import time
import pyarrow as pa
from dora import Node

def main():
    node = Node("text-segmenter")
    print("[Segmenter] Started - will pass through all text immediately")
    
    segment_index = 0
    
    for event in node:
        if event["type"] == "INPUT":
            if event["id"] == "text":
                text = event["value"][0].as_py()
                metadata = event.get("metadata", {})
                
                print(f"[Segmenter] Received text: {len(text)} chars")
                print(f"[Segmenter] Text preview: {text[:100]}...")
                
                # Immediately send as segment
                out_metadata = {
                    "segment_index": segment_index,
                    "original_metadata": metadata
                }
                
                node.send_output(
                    "text_segment",
                    pa.array([text]),
                    metadata=out_metadata
                )
                
                print(f"[Segmenter] → Sent segment {segment_index}: {len(text)} chars")
                segment_index += 1
                
                # Also send completion signal
                node.send_output(
                    "status",
                    pa.array(["segment_sent"]),
                    metadata={"segment_index": segment_index - 1}
                )
                
            elif event["id"] == "tts_complete":
                print(f"[Segmenter] TTS completed")
                
        elif event["type"] == "STOP":
            break
    
    print("[Segmenter] Stopped")

if __name__ == "__main__":
    main()