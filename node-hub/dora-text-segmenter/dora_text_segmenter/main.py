#!/usr/bin/env python3
"""
Text Segmenter Node - Sequential segment sender.
Segments text by punctuation and sends segments ONE AT A TIME.
Waits for control signal before sending next segment when using backpressure.
"""

import os
import re
import time
from typing import List, Dict, Any
import pyarrow as pa
from dora import Node


def send_log(node: Node, level: str, message: str):
    """Send log message through node output."""
    try:
        node.send_output(
            "status",
            pa.array([f"[{level}] {message}"]),
            metadata={"log_level": level, "timestamp": time.time()}
        )
    except:
        # Fallback to print if sending fails
        print(f"[{level}] {message}")


# Fixed long test text
TEST_TEXT = """
人工智能正在彻底改变我们的世界。从自动驾驶汽车到智能家居，从医疗诊断到金融分析，AI的应用无处不在。
深度学习技术的突破，让机器能够理解和生成人类语言，识别图像和声音，甚至创作艺术作品。
语音合成技术也在飞速发展。现代的TTS系统不仅能够生成自然流畅的语音，还能模拟不同的情感和语调。
这种技术的应用场景非常广泛：智能助手、有声读物、语音导航、无障碍服务等等。
在教育领域，TTS可以帮助学生更好地学习语言，特别是对于视觉障碍者来说，这项技术尤为重要。
商业应用中，客服机器人使用TTS技术提供24小时服务，大大提高了效率。
娱乐产业也在积极采用这项技术，游戏中的NPC可以拥有独特的声音，动画角色能够说出流利的对话。
未来，随着技术的不断进步，我们将看到更加逼真、更加个性化的语音合成系统。
情感计算的融入，将使得机器生成的语音更具表现力，能够传达微妙的情绪变化。
多语言、多方言的支持，将打破语言障碍，促进全球交流。
实时语音转换技术，可以让人们用自己的声音说出不同的语言。
这些创新将深刻影响人机交互的方式，让技术变得更加人性化。
让我们一起期待这个充满可能的未来！
"""


class SequentialTextSegmenter:
    """Segments text and sends segments sequentially."""
    
    def __init__(self):
        # Configuration
        self.max_length = int(os.getenv("MAX_SEGMENT_LENGTH", "150"))
        self.min_length = int(os.getenv("MIN_SEGMENT_LENGTH", "50"))
        self.use_test_text = os.getenv("USE_TEST_TEXT", "false").lower() == "true"
        self.use_backpressure = os.getenv("USE_BACKPRESSURE_CONTROL", "false").lower() == "true"
        
        # Session state
        self.current_session = None
        self.session_start_time = None
        self.is_paused = False  # Track if we're paused by controller
        
    def segment_by_punctuation(self, text: str) -> List[str]:
        """
        Segment text by punctuation marks, respecting sentence boundaries.
        Never cuts in the middle of a sentence.
        """
        # Clean up text
        text = text.strip()
        text = re.sub(r'\s+', ' ', text)
        
        # Split by sentence-ending punctuation (keeps punctuation)
        sentence_endings = r'([。！？.!?])'
        parts = re.split(sentence_endings, text)
        
        # Reconstruct complete sentences with their punctuation
        sentences = []
        for i in range(0, len(parts), 2):
            sentence = parts[i].strip()
            if i + 1 < len(parts):
                sentence += parts[i + 1]  # Add punctuation
            if sentence and not sentence.isspace():
                sentences.append(sentence)
        
        # Now combine sentences into segments of appropriate length
        segments = []
        current_segment = ""
        
        for sentence in sentences:
            sentence = sentence.strip()
            if not sentence:
                continue
            
            # If current segment is empty, start with this sentence
            if not current_segment:
                current_segment = sentence
            
            # If adding this sentence keeps us under max, add it
            elif len(current_segment) + len(sentence) <= self.max_length:
                current_segment += sentence
            
            # If current segment is long enough, save it and start new
            elif len(current_segment) >= self.min_length:
                segments.append(current_segment)
                current_segment = sentence
            
            # Current segment too short but adding would exceed max
            # This is rare - sentence is very long
            else:
                # If the sentence alone is too long, we need to split it
                if len(sentence) > self.max_length:
                    # First, save current segment if it's long enough
                    if len(current_segment) >= self.min_length:
                        segments.append(current_segment)
                        current_segment = ""
                    
                    # Split long sentence by commas/semicolons
                    clause_marks = r'([，,；;、])'
                    clause_parts = re.split(clause_marks, sentence)
                    
                    # Reconstruct clauses
                    clauses = []
                    for j in range(0, len(clause_parts), 2):
                        clause = clause_parts[j].strip()
                        if j + 1 < len(clause_parts):
                            clause += clause_parts[j + 1]
                        if clause:
                            clauses.append(clause)
                    
                    # Build segments from clauses
                    for clause in clauses:
                        if not current_segment:
                            current_segment = clause
                        elif len(current_segment) + len(clause) <= self.max_length:
                            current_segment += clause
                        else:
                            if len(current_segment) >= self.min_length:
                                segments.append(current_segment)
                            current_segment = clause
                else:
                    # Sentence is not too long, just add current and start new
                    current_segment += sentence
                    if len(current_segment) >= self.min_length:
                        segments.append(current_segment)
                        current_segment = ""
        
        # Add final segment if long enough
        if current_segment and len(current_segment) >= self.min_length:
            segments.append(current_segment)
        elif current_segment and segments:
            # If last bit is too short, append to previous segment if possible
            if len(segments[-1]) + len(current_segment) <= self.max_length * 1.2:  # Allow slight overflow
                segments[-1] += current_segment
            else:
                # Force add even if short to not lose content
                segments.append(current_segment)
        elif current_segment:
            # Only segment and it's short - still add it
            segments.append(current_segment)
        
        # Final validation - ensure no segments are cut mid-sentence
        validated_segments = []
        for seg in segments:
            # Check if segment ends properly
            if seg.endswith(('。', '！', '？', '.', '!', '?', '，', ',', '；', ';', '、')):
                validated_segments.append(seg)
            else:
                # Try to find the last complete sentence
                last_punct = max(
                    seg.rfind('。'), seg.rfind('！'), seg.rfind('？'),
                    seg.rfind('.'), seg.rfind('!'), seg.rfind('?')
                )
                if last_punct > 0:
                    # Split at last sentence boundary
                    validated_segments.append(seg[:last_punct + 1])
                    # Don't lose the remainder
                    remainder = seg[last_punct + 1:].strip()
                    if remainder and validated_segments:
                        # Add to next segment if exists
                        if len(validated_segments) > 0:
                            validated_segments[-1] += remainder
                else:
                    # No sentence boundary found, keep as is
                    validated_segments.append(seg)
        
        return validated_segments if validated_segments else segments
    
    def start_new_session(self, node: Node, text: str = None):
        """Start a new segmentation session."""
        # Use test text if configured or no text provided
        if self.use_test_text or text is None:
            text = TEST_TEXT
            send_log(node, "INFO", f"Using test text ({len(text)} chars)")
        else:
            send_log(node, "INFO", f"Using provided text ({len(text)} chars)")
        
        # Segment the text
        segments = self.segment_by_punctuation(text)
        
        # Initialize session
        self.current_session = {
            "segments": segments,
            "current_index": 0,
            "session_id": f"session_{int(time.time())}",
            "total_sent": 0
        }
        self.session_start_time = time.time()
        self.is_paused = False  # Start unpaused
        
        send_log(node, "INFO", f"Started new session: {self.current_session['session_id']}")
        send_log(node, "INFO", f"Text length: {len(text)} chars")
        send_log(node, "INFO", f"Segmented into {len(segments)} parts")
        send_log(node, "INFO", f"Segment lengths: {[len(s) for s in segments]}")
        send_log(node, "INFO", f"First 3 segments:")
        for i in range(min(3, len(segments))):
            send_log(node, "INFO", f"  {i+1}. [{len(segments[i])} chars] {segments[i]}")
        
        # IMPORTANT: Send first segment immediately to avoid deadlock
        send_log(node, "INFO", "Sending first segment to start the flow")
        self.send_next_segment(node)
    
    def send_next_segment(self, node: Node):
        """Send the next segment in the current session."""
        if not self.current_session:
            return False
        
        session = self.current_session
        
        # Check if there are more segments
        if session["current_index"] < len(session["segments"]):
            segment = session["segments"][session["current_index"]]
            
            # Send segment to TTS
            metadata = {
                "session_id": session["session_id"],
                "segment_index": session["current_index"],
                "total_segments": len(session["segments"]),
                "is_first": session["current_index"] == 0,
                "is_last": session["current_index"] == len(session["segments"]) - 1,
                "segment_length": len(segment)
            }
            
            node.send_output(
                "text_segment",
                pa.array([segment]),
                metadata=metadata
            )
            
            elapsed = time.time() - self.session_start_time
            # Show full segment to verify it's not cut
            if len(segment) > 60:
                display = f"{segment[:30]}...{segment[-20:]}"
            else:
                display = segment
            send_log(node, "INFO", f"[{elapsed:.1f}s] → Sent segment {session['current_index'] + 1}/{len(session['segments'])}: {display}")
            
            session["current_index"] += 1
            session["total_sent"] += 1
            return True
            
        else:
            # All segments sent
            elapsed = time.time() - self.session_start_time
            send_log(node, "INFO", "Session complete!")
            send_log(node, "INFO", f"  Total segments: {len(session['segments'])}")
            send_log(node, "INFO", f"  Total time: {elapsed:.2f}s")
            if len(session['segments']) > 0:
                send_log(node, "INFO", f"  Average time per segment: {elapsed/len(session['segments']):.2f}s")
            
            # Send completion status
            node.send_output(
                "status",
                pa.array(["session_complete"]),
                metadata={
                    "session_id": session["session_id"],
                    "total_segments": len(session["segments"]),
                    "total_time": elapsed
                }
            )
            
            # Reset for potential new session
            self.current_session = None
            return False


def main():
    """Main entry point for sequential text segmenter."""
    node = Node("text-segmenter")
    segmenter = SequentialTextSegmenter()
    
    send_log(node, "INFO", "=" * 60)
    send_log(node, "INFO", "Sequential Text Segmenter Started")
    send_log(node, "INFO", "=" * 60)
    send_log(node, "INFO", f"Max segment length: {segmenter.max_length} chars")
    send_log(node, "INFO", f"Min segment length: {segmenter.min_length} chars")
    send_log(node, "INFO", f"Using test text: {segmenter.use_test_text}")
    send_log(node, "INFO", f"Backpressure control: {segmenter.use_backpressure}")
    if segmenter.use_backpressure:
        send_log(node, "INFO", "Control mode: Waiting for controller signals")
    else:
        send_log(node, "INFO", "Control mode: Automatic (no flow control)")
    send_log(node, "INFO", "=" * 60)
    
    # Only start with test text if configured
    if segmenter.use_test_text:
        send_log(node, "INFO", "Starting automatic session with test text...")
        segmenter.start_new_session(node)
    else:
        send_log(node, "INFO", "Waiting for external text input...")
    
    while True:
        event = node.next(timeout=1.0)
        
        if event is None:
            continue
        
        if event["type"] == "INPUT":
            if event["id"] == "text":
                # Received external text input
                text = event["value"][0].as_py()
                metadata = event.get("metadata", {})
                
                send_log(node, "INFO", f"Received external text ({len(text)} chars)")
                send_log(node, "INFO", f"Source: {metadata.get('text_type', 'unknown')}")
                
                # Start new session with external text
                segmenter.start_new_session(node, text)
                
            elif event["id"] == "tts_complete":
                # TTS has completed processing a segment
                completion_status = event["value"][0].as_py()
                metadata = event.get("metadata", {})
                segment_index = metadata.get("segment_index", -1)
                session_id = metadata.get("session_id", "")
                
                send_log(node, "INFO", f"TTS completed segment {segment_index + 1}")
                
                # Check if we should send next segment
                # Conditions: have session, not paused, and matches current session
                if segmenter.current_session:
                    current_session_id = segmenter.current_session.get("session_id", "")
                    if session_id != current_session_id:
                        send_log(node, "INFO", "Ignoring completion from old session")
                    elif segmenter.is_paused:
                        send_log(node, "INFO", "TTS complete but PAUSED - holding next segment")
                    else:
                        # Send next segment
                        if not segmenter.send_next_segment(node):
                            send_log(node, "INFO", "No more segments to send")
                        
            elif event["id"] == "control_signal":
                # Control signal from conversation controller (backpressure)
                signal = event["value"][0].as_py()
                metadata = event.get("metadata", {})
                buffer_percentage = metadata.get("buffer_percentage", 0)
                
                send_log(node, "INFO", f"Control signal: '{signal}' (buffer: {buffer_percentage:.1f}%)")
                
                # Handle control signals
                if signal == "pause":
                    segmenter.is_paused = True
                    send_log(node, "INFO", "PAUSED by backpressure - buffer full")
                    
                elif signal == "continue":
                    # Resume if paused
                    was_paused = segmenter.is_paused
                    segmenter.is_paused = False
                    
                    if was_paused:
                        send_log(node, "INFO", "RESUMED by backpressure - buffer has space")
                        # Note: We don't send immediately - wait for TTS completion
                        
            elif event["id"] == "control":
                # Control commands
                command = event["value"][0].as_py()
                
                if command == "reset":
                    # Clear all unsent segments and reset state
                    if segmenter.current_session:
                        session = segmenter.current_session
                        unsent_count = len(session["segments"]) - session["current_index"]
                        send_log(node, "INFO", f"RESET received - clearing {unsent_count} unsent segments")
                        send_log(node, "INFO", f"  Session ID: {session['session_id']}")
                        send_log(node, "INFO", f"  Segments sent: {session['current_index']}/{len(session['segments'])}")
                        send_log(node, "INFO", f"  Segments cleared: {unsent_count}")
                    else:
                        send_log(node, "INFO", "RESET received - no active session")
                    
                    # Reset all state
                    segmenter.current_session = None
                    segmenter.is_paused = False
                    segmenter.session_start_time = None
                    
                    # Send reset confirmation
                    node.send_output(
                        "status",
                        pa.array(["reset_complete"]),
                        metadata={"timestamp": time.time()}
                    )
                    send_log(node, "INFO", "Reset complete - ready for new session")
                
                elif command == "status":
                    if segmenter.current_session:
                        session = segmenter.current_session
                        send_log(node, "INFO", f"Active session: {session['session_id']}")
                        send_log(node, "INFO", f"  Progress: {session['current_index']}/{len(session['segments'])}")
                        send_log(node, "INFO", f"  Sent: {session['total_sent']}")
                        send_log(node, "INFO", f"  Paused: {segmenter.is_paused}")
                    else:
                        send_log(node, "INFO", "No active session")
        
        elif event["type"] == "STOP":
            break
    
    send_log(node, "INFO", "Text segmenter stopped")


if __name__ == "__main__":
    main()