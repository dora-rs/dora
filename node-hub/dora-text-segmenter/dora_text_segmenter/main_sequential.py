#!/usr/bin/env python3
"""
Text Segmenter Node - Sequential segment sender.
Segments text by punctuation and sends segments ONE AT A TIME.
Waits for TTS completion before sending next segment.
"""

import os
import re
import time
import json
from typing import List, Dict, Any
import pyarrow as pa
from dora import Node


class SequentialTextSegmenter:
    """Segments text and sends segments sequentially after TTS completion."""
    
    def __init__(self):
        # Configuration
        self.max_length = int(os.getenv("MAX_SEGMENT_LENGTH", "50"))
        self.min_length = int(os.getenv("MIN_SEGMENT_LENGTH", "3"))
        
        # State for each session
        self.sessions: Dict[str, Dict[str, Any]] = {}
        
    def segment_by_punctuation(self, text: str) -> List[str]:
        """Segment text by punctuation marks."""
        # Define punctuation patterns
        sentence_marks = r'[。！？.!?]'
        clause_marks = r'[；;]'
        phrase_marks = r'[，,、]'
        
        segments = []
        
        # Split by sentence-ending punctuation
        parts = re.split(f'({sentence_marks})', text)
        
        current_segment = ""
        for i in range(0, len(parts), 2):
            if i + 1 < len(parts):
                segment = parts[i] + parts[i + 1]
            else:
                segment = parts[i]
            
            segment = segment.strip()
            if not segment:
                continue
            
            # Check length and split if needed
            if len(segment) > self.max_length:
                # Further split by clause marks
                clause_parts = re.split(f'({clause_marks})', segment)
                for j in range(0, len(clause_parts), 2):
                    if j + 1 < len(clause_parts):
                        clause = clause_parts[j] + clause_parts[j + 1]
                    else:
                        clause = clause_parts[j]
                    
                    clause = clause.strip()
                    if clause and len(clause) >= self.min_length:
                        segments.append(clause)
            else:
                # Combine short segments
                if current_segment and len(current_segment) + len(segment) <= self.max_length:
                    current_segment += segment
                else:
                    if current_segment:
                        segments.append(current_segment)
                    current_segment = segment
        
        if current_segment:
            segments.append(current_segment)
        
        return segments
    
    def send_next_segment(self, node: Node, session_id: str):
        """Send the next segment for a session."""
        if session_id not in self.sessions:
            return
        
        session = self.sessions[session_id]
        
        # Check if there are more segments to send
        if session["current_index"] < len(session["segments"]):
            segment = session["segments"][session["current_index"]]
            
            # Send segment
            metadata = {
                "session_id": session_id,
                "request_id": session["request_id"],
                "segment_index": session["current_index"],
                "total_segments": len(session["segments"]),
                "is_first": session["current_index"] == 0,
                "is_last": session["current_index"] == len(session["segments"]) - 1,
                "segment_text": segment
            }
            
            node.send_output(
                "text_segment",
                pa.array([segment]),
                metadata=metadata
            )
            
            print(f"[Segmenter] Sent segment {session['current_index'] + 1}/{len(session['segments'])}: {segment[:30]}...")
            
            # Mark as sent
            session["current_index"] += 1
            session["awaiting_completion"] = True
            session["last_sent_time"] = time.time()
        else:
            # All segments sent
            print(f"[Segmenter] Session {session_id} complete - all segments sent")
            
            # Send completion status
            node.send_output(
                "status",
                pa.array(["all_segments_sent"]),
                metadata={
                    "session_id": session_id,
                    "total_segments": len(session["segments"])
                }
            )
            
            # Clean up session
            del self.sessions[session_id]


def main():
    """Main entry point for sequential text segmenter."""
    node = Node("text-segmenter")
    segmenter = SequentialTextSegmenter()
    
    print("[Sequential Text Segmenter] Started")
    print(f"[Sequential Text Segmenter] Max segment length: {segmenter.max_length}")
    
    while True:
        event = node.next(timeout=0.5)
        
        # Check for timeouts
        current_time = time.time()
        for session_id, session in list(segmenter.sessions.items()):
            if session.get("awaiting_completion", False):
                if current_time - session.get("last_sent_time", 0) > 30:
                    print(f"[Segmenter] Timeout for session {session_id}, forcing next segment")
                    session["awaiting_completion"] = False
                    segmenter.send_next_segment(node, session_id)
        
        if event is None:
            continue
        
        if event["type"] == "INPUT":
            if event["id"] == "text":
                # New text to segment
                text = event["value"][0].as_py()
                metadata = event.get("metadata", {})
                session_id = metadata.get("session_id", f"session_{time.time()}")
                request_id = metadata.get("request_id", f"req_{time.time()}")
                
                print(f"\n[Segmenter] New text received ({len(text)} chars) for session {session_id}")
                
                # Segment the text
                segments = segmenter.segment_by_punctuation(text)
                print(f"[Segmenter] Segmented into {len(segments)} parts")
                
                # Initialize session
                segmenter.sessions[session_id] = {
                    "segments": segments,
                    "current_index": 0,
                    "request_id": request_id,
                    "awaiting_completion": False,
                    "last_sent_time": 0,
                    "start_time": time.time()
                }
                
                # Send first segment immediately
                segmenter.send_next_segment(node, session_id)
                
            elif event["id"] == "tts_complete":
                # TTS completed a segment, send next one
                metadata = event.get("metadata", {})
                session_id = metadata.get("session_id")
                segment_index = metadata.get("segment_index", -1)
                
                print(f"[Segmenter] TTS completed segment {segment_index + 1} for session {session_id}")
                
                if session_id in segmenter.sessions:
                    session = segmenter.sessions[session_id]
                    session["awaiting_completion"] = False
                    
                    # Send next segment
                    segmenter.send_next_segment(node, session_id)
                    
        elif event["type"] == "STOP":
            print("[Sequential Text Segmenter] Stopping...")
            break
    
    print("[Sequential Text Segmenter] Stopped")


if __name__ == "__main__":
    main()