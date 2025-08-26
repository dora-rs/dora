#!/usr/bin/env python3
"""
Queue-based Text Segmenter
1. No deadlock - first segment sent immediately
2. Don't judge if segments are complete - just queue them
3. Send one at a time, triggered by TTS completion
4. Skip segments with only punctuation or numbers
"""

import time
import re
import pyarrow as pa
from dora import Node
from collections import deque

def should_skip_segment(text):
    """Check if segment should be skipped (only punctuation or numbers)"""
    # Remove whitespace for checking
    text_stripped = text.strip()
    
    # Skip if empty
    if not text_stripped:
        return True
    
    # Pattern: only punctuation, numbers, whitespace, or common symbols
    # Includes Chinese and English punctuation
    skip_pattern = r'^[\s\d\.\,\!\?\;\:\-\—\~\@\#\$\%\^\&\*\(\)\[\]\{\}\_\+\=\|\\\/\<\>\"\'\`。，！？；：、""''（）【】《》「」『』〈〉〔〕……——～·]+$'
    
    if re.match(skip_pattern, text_stripped):
        print(f"[Segmenter] Skipping punctuation/number only segment: '{text_stripped}'")
        return True
    
    return False

def main():
    node = Node("text-segmenter")
    
    # Simple queue for segments
    segment_queue = deque()
    is_sending = False
    segment_index = 0
    
    print("[Segmenter] Started - Queue-based segmenter")
    print("[Segmenter] Will send first segment immediately, then wait for TTS completion")
    print("[Segmenter] Will skip segments with only punctuation or numbers")
    
    for event in node:
        if event["type"] == "INPUT":
            if event["id"] == "text":
                # Received text from LLM
                text = event["value"][0].as_py()
                metadata = event.get("metadata", {})
                
                print(f"[Segmenter] Received text chunk: {len(text)} chars")
                
                # Check if we should skip this segment
                if not should_skip_segment(text):
                    # Valid segment - add to queue
                    segment_queue.append({
                        "text": text,
                        "metadata": metadata,
                        "index": len(segment_queue)
                    })
                    
                    print(f"[Segmenter] Queued segment, queue size: {len(segment_queue)}")
                
                # Try to send a segment if not currently sending
                # This happens whether we queued the current segment or skipped it
                # Ensures no deadlock even if first segments are all punctuation
                if not is_sending and segment_queue:
                    segment = segment_queue.popleft()
                    
                    # Send segment to TTS
                    node.send_output(
                        "text_segment",
                        pa.array([segment["text"]]),
                        metadata={
                            "segment_index": segment_index,
                            "queue_remaining": len(segment_queue),
                            **segment["metadata"]
                        }
                    )
                    
                    print(f"[Segmenter] → Sent segment {segment_index}: '{segment['text'][:30]}...' ({len(segment['text'])} chars)")
                    segment_index += 1
                    is_sending = True
                    
            elif event["id"] == "tts_complete":
                # TTS completed a segment
                print(f"[Segmenter] TTS completed, queue size: {len(segment_queue)}")
                
                # Send next segment if available
                if segment_queue:
                    segment = segment_queue.popleft()
                    
                    node.send_output(
                        "text_segment",
                        pa.array([segment["text"]]),
                        metadata={
                            "segment_index": segment_index,
                            "queue_remaining": len(segment_queue),
                            **segment["metadata"]
                        }
                    )
                    
                    print(f"[Segmenter] → Sent segment {segment_index}: '{segment['text'][:30]}...' ({len(segment['text'])} chars)")
                    segment_index += 1
                else:
                    # No more segments to send
                    print("[Segmenter] No more segments in queue")
                    is_sending = False
                    
            elif event["id"] == "control":
                # Reset command
                command = event["value"][0].as_py()
                if command == "reset":
                    print(f"[Segmenter] RESET - clearing {len(segment_queue)} queued segments")
                    segment_queue.clear()
                    is_sending = False
                    segment_index = 0
                    
        elif event["type"] == "STOP":
            break
    
    print("[Segmenter] Stopped")

if __name__ == "__main__":
    main()