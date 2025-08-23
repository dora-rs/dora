#!/usr/bin/env python3
"""
Chat Controller Node - Static node that orchestrates voice chatbot conversation flow.
Accumulates transcriptions, sends questions to LLM, and manages speech monitor pause/resume.
LLM responses go directly to text segmenter (not through this controller).
"""

import os
import time
import json
import pyarrow as pa
from dora import Node
from typing import List, Dict, Any, Optional


class ChatController:
    """
    Manages conversation state and controls flow between input and output.
    """
    
    def __init__(self):
        # Load configuration from environment
        self.min_question_length = int(os.getenv("MIN_QUESTION_LENGTH", "5"))
        self.max_history = int(os.getenv("MAX_CONVERSATION_HISTORY", "10"))
        self.answer_complete_buffer_threshold = float(os.getenv("ANSWER_COMPLETE_BUFFER_THRESHOLD", "1.0"))
        
        # State management
        self.state = "listening"  # listening, processing, answering
        self.current_transcriptions = []
        self.conversation_history = []
        self.current_question = ""
        
        # Timing
        self.question_start_time = None
        self.answer_start_time = None
        
        # Statistics
        self.questions_processed = 0
        self.total_response_time = 0
        self.transcription_count = 0
        
    def send_log(self, node: Node, level: str, message: str):
        """Send log message through node output."""
        try:
            node.send_output(
                "log",
                pa.array([f"[{level}] [Chat Controller] {message}"]),
                metadata={"log_level": level, "timestamp": time.time()}
            )
        except:
            # Fallback to print if sending fails
            print(f"[{level}] [Chat Controller] {message}")
            
    def on_speech_started(self, node: Node, event: Dict[str, Any]):
        """Handle speech start signal from Speech Monitor."""
        if self.state == "answering":
            self.send_log(node, "DEBUG", "Ignoring speech_started during answer playback")
            return
            
        if self.state == "listening":
            self.send_log(node, "INFO", "User started speaking")
            # Clear transcriptions for new question
            self.current_transcriptions = []
            self.question_start_time = time.time()
            
    def on_speech_ended(self, node: Node, event: Dict[str, Any]):
        """Handle speech end signal - might be end of a sentence."""
        if self.state == "answering":
            self.send_log(node, "DEBUG", "Ignoring speech_ended during answer playback")
            return
            
        if self.state == "listening":
            metadata = event.get("metadata", {})
            duration = metadata.get("duration", 0)
            
            # Log the speech ended event but don't act on it immediately
            # Wait for question_ended signal for more definitive end
            if self.current_transcriptions:
                partial_question = " ".join(self.current_transcriptions).strip()
                self.send_log(node, "DEBUG", f"Speech segment ended ({duration:.1f}s), partial: {partial_question[:50]}...")
            else:
                self.send_log(node, "DEBUG", "Speech ended but no transcriptions accumulated")
    
    def on_question_ended(self, node: Node, event: Dict[str, Any]):
        """Handle question end signal - longer silence detected, question is complete!"""
        if self.state == "answering":
            self.send_log(node, "DEBUG", "Ignoring question_ended during answer playback")
            return
            
        if self.state == "listening":
            metadata = event.get("metadata", {})
            silence_duration_ms = metadata.get("silence_duration_ms", 0)
            last_segment = metadata.get("last_segment", 0)
            
            # Join all accumulated transcriptions
            if self.current_transcriptions:
                full_question = " ".join(self.current_transcriptions).strip()
                
                # Check minimum length
                if len(full_question) >= self.min_question_length:
                    self.send_log(node, "INFO", f"Question complete (silence: {silence_duration_ms:.0f}ms): {full_question}")
                    
                    # Store question
                    self.current_question = full_question
                    
                    # Transition to processing
                    self.state = "processing"
                    self.questions_processed += 1
                    
                    # Pause speech monitor
                    self.pause_speech_monitor(node)
                    
                    # Send question to LLM
                    self.send_question_to_llm(node, full_question)
                    
                    # Clear transcriptions
                    self.current_transcriptions = []
                else:
                    self.send_log(node, "INFO", f"Question too short ({len(full_question)} chars), waiting for more")
            else:
                self.send_log(node, "DEBUG", "Question ended but no transcriptions accumulated")
                
    def on_transcription(self, node: Node, event: Dict[str, Any]):
        """
        Process transcriptions - accumulate during listening, drop during answering.
        """
        text = event["value"][0].as_py()
        
        if self.state == "listening":
            if text and text.strip():
                self.current_transcriptions.append(text.strip())
                self.transcription_count += 1
                self.send_log(node, "DEBUG", f"Transcription #{self.transcription_count}: {text}")
                
                # Optionally route transcription for display/monitoring
                node.send_output(
                    "transcription_display",
                    pa.array([text]),
                    metadata={
                        "state": self.state,
                        "accumulated": len(self.current_transcriptions)
                    }
                )
                
        elif self.state in ["processing", "answering"]:
            self.send_log(node, "DEBUG", f"Dropping transcription in {self.state} state: {text}")
            
    def on_buffer_status(self, node: Node, event: Dict[str, Any]):
        """Monitor audio buffer to detect when answer playback is complete."""
        if self.state != "answering":
            return
            
        buffer_percentage = event["value"][0].as_py()
        metadata = event.get("metadata", {})
        buffer_seconds = metadata.get("buffer_seconds", 0)
        
        # No rate limiting needed - audio player only sends once per second anyway
        
        # Log buffer status for debugging
        self.send_log(node, "DEBUG", f"Buffer status: {buffer_percentage:.1f}% (threshold: {self.answer_complete_buffer_threshold}%)")
        
        # Check if buffer is nearly empty (answer finished playing)
        if buffer_percentage < self.answer_complete_buffer_threshold:
            answer_duration = time.time() - self.answer_start_time if self.answer_start_time else 0
            self.send_log(node, "INFO", f"Answer playback complete ({answer_duration:.1f}s, buffer: {buffer_percentage:.1f}%)")
            
            # Transition back to listening
            self.state = "listening"
            self.transcription_count = 0
            
            # Resume speech monitor
            self.resume_speech_monitor(node)
            
            # Send status update
            node.send_output(
                "status",
                pa.array(["ready_for_question"]),
                metadata={
                    "questions_processed": self.questions_processed,
                    "state": self.state,
                    "conversation_length": len(self.conversation_history)
                }
            )
        else:
            # Still playing
            if buffer_seconds > 0:
                self.send_log(node, "DEBUG", f"Answer playing... (buffer: {buffer_percentage:.1f}%, {buffer_seconds:.1f}s remaining)")
            
    def pause_speech_monitor(self, node: Node):
        """Send pause command to speech monitor."""
        self.send_log(node, "INFO", "Pausing speech monitor")
        node.send_output(
            "speech_control",
            pa.array(["pause"]),
            metadata={"timestamp": time.time()}
        )
        
    def resume_speech_monitor(self, node: Node):
        """Send resume command to speech monitor."""
        self.send_log(node, "INFO", "Resuming speech monitor")
        node.send_output(
            "speech_control",
            pa.array(["resume"]),
            metadata={"timestamp": time.time()}
        )
        
    def send_question_to_llm(self, node: Node, question: str):
        """Send the complete question to LLM (response will go directly to text segmenter)."""
        # Build context from recent history
        context_messages = []
        for entry in self.conversation_history[-self.max_history:]:
            if entry["role"] == "user":
                context_messages.append(f"User: {entry['content']}")
            else:
                context_messages.append(f"Assistant: {entry['content']}")
        
        # Create prompt - system prompt is handled by Qwen3 node configuration
        if context_messages:
            # Include conversation history for context
            context_str = "\n".join(context_messages)
            prompt = f"""Previous conversation:
{context_str}

User: {question}"""
        else:
            # First question - just send the question
            prompt = question
        
        self.send_log(node, "INFO", f"Sending question to LLM (with {len(self.conversation_history)} history items)")
        
        # Send to LLM
        node.send_output(
            "question_to_llm",  # This goes to Qwen3 LLM
            pa.array([prompt]),
            metadata={
                "timestamp": time.time(),
                "question_num": self.questions_processed,
                "has_context": len(self.conversation_history) > 0,
                "raw_question": question
            }
        )
        
        # Transition to answering state
        # Note: LLM response goes directly to text segmenter, not back to us
        self.state = "answering"
        self.answer_start_time = time.time()
        self.send_log(node, "INFO", f"Transitioned to ANSWERING state, monitoring buffer for completion")
        
        # Add question to conversation history
        self.conversation_history.append({
            "role": "user",
            "content": self.current_question,
            "timestamp": self.question_start_time
        })
        
        # Note: We'll add the assistant's response next time when we have context
        # Or we could monitor the LLM output separately if needed
        
        # Trim history if getting too long
        if len(self.conversation_history) > self.max_history * 2:
            self.conversation_history = self.conversation_history[-(self.max_history * 2):]
        
        self.send_log(node, "INFO", "Question sent to LLM, waiting for answer playback to complete...")
        
    def on_control(self, node: Node, event: Dict[str, Any]):
        """Handle control commands."""
        command = event["value"][0].as_py()
        
        if command == "reset":
            self.send_log(node, "INFO", "Resetting conversation")
            self.conversation_history = []
            self.current_transcriptions = []
            self.state = "listening"
            self.questions_processed = 0
            
            # Resume speech monitor in case it was paused
            self.resume_speech_monitor(node)
            
        elif command == "status":
            status = {
                "state": self.state,
                "questions_processed": self.questions_processed,
                "history_length": len(self.conversation_history),
                "current_transcriptions": len(self.current_transcriptions)
            }
            self.send_log(node, "INFO", f"Status: {json.dumps(status)}")
            
            node.send_output(
                "status",
                pa.array([json.dumps(status)]),
                metadata={"timestamp": time.time()}
            )


def main():
    """Main entry point for chat controller static node."""
    # Initialize node (static nodes don't need ID)
    node = Node()
    controller = ChatController()
    
    # Send initial log messages
    controller.send_log(node, "INFO", "=" * 60)
    controller.send_log(node, "INFO", "CHAT CONTROLLER STARTED")
    controller.send_log(node, "INFO", f"Min question length: {controller.min_question_length} chars")
    controller.send_log(node, "INFO", f"Max conversation history: {controller.max_history} exchanges")
    controller.send_log(node, "INFO", f"Answer complete threshold: {controller.answer_complete_buffer_threshold}%")
    controller.send_log(node, "INFO", "=" * 60)
    controller.send_log(node, "INFO", "Ready for conversation...")
    
    for event in node:
        try:
            if event["type"] == "INPUT":
                input_id = event["id"]
                
                if input_id == "speech_started":
                    controller.on_speech_started(node, event)
                    
                elif input_id == "speech_ended":
                    controller.on_speech_ended(node, event)
                    
                elif input_id == "question_ended":
                    controller.on_question_ended(node, event)
                    
                elif input_id == "transcription":
                    controller.on_transcription(node, event)
                    
                elif input_id == "buffer_status":
                    controller.on_buffer_status(node, event)
                    
                elif input_id == "control":
                    controller.on_control(node, event)
                    
            elif event["type"] == "STOP":
                break
                
        except Exception as e:
            controller.send_log(node, "ERROR", f"Error processing event: {e}")
            
    controller.send_log(node, "INFO", f"Stopped after {controller.questions_processed} questions")


if __name__ == "__main__":
    main()