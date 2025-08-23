#!/usr/bin/env python3
"""
Chat Controller Node - Orchestrates voice chatbot conversation flow.
Manages pause/resume of speech monitor and coordinates between ASR and TTS.
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
        self.answer_complete_buffer_threshold = float(os.getenv("ANSWER_COMPLETE_BUFFER_THRESHOLD", "5.0"))
        
        # State management
        self.state = "listening"  # listening, processing, answering
        self.current_transcriptions = []
        self.conversation_history = []
        self.current_question = ""
        self.current_answer = ""
        
        # Timing
        self.question_start_time = None
        self.answer_start_time = None
        self.last_buffer_check_time = 0
        
        # Statistics
        self.questions_processed = 0
        self.total_response_time = 0
        self.transcription_count = 0
        
    def log(self, level: str, message: str):
        """Log message with consistent format."""
        print(f"[Chat Controller] [{level}] {message}")
        
    def on_speech_started(self, node: Node, event: Dict[str, Any]):
        """Handle speech start signal from Speech Monitor."""
        if self.state == "answering":
            self.log("DEBUG", "Ignoring speech_started during answer playback")
            return
            
        if self.state == "listening":
            self.log("INFO", "User started speaking")
            # Clear transcriptions for new question
            self.current_transcriptions = []
            self.question_start_time = time.time()
            
    def on_speech_ended(self, node: Node, event: Dict[str, Any]):
        """Handle speech end signal - question is complete!"""
        if self.state == "answering":
            self.log("DEBUG", "Ignoring speech_ended during answer playback")
            return
            
        if self.state == "listening":
            metadata = event.get("metadata", {})
            duration = metadata.get("duration", 0)
            
            # Join all accumulated transcriptions
            if self.current_transcriptions:
                full_question = " ".join(self.current_transcriptions).strip()
                
                # Check minimum length
                if len(full_question) >= self.min_question_length:
                    self.log("INFO", f"Question complete ({duration:.1f}s): {full_question}")
                    
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
                    self.log("INFO", f"Question too short ({len(full_question)} chars), waiting for more")
            else:
                self.log("DEBUG", "Speech ended but no transcriptions accumulated")
                
    def on_transcription(self, node: Node, event: Dict[str, Any]):
        """Accumulate transcriptions while listening."""
        text = event["value"][0].as_py()
        
        if self.state == "listening":
            if text and text.strip():
                self.current_transcriptions.append(text.strip())
                self.transcription_count += 1
                self.log("DEBUG", f"Transcription #{self.transcription_count}: {text}")
                
        elif self.state in ["processing", "answering"]:
            self.log("DEBUG", f"Ignoring transcription in {self.state} state")
            
    def on_llm_response(self, node: Node, event: Dict[str, Any]):
        """Handle response from LLM."""
        if self.state != "processing":
            self.log("WARNING", f"Unexpected LLM response in {self.state} state")
            return
            
        response = event["value"][0].as_py()
        self.log("INFO", f"LLM response received ({len(response)} chars)")
        
        # Store answer
        self.current_answer = response
        
        # Add to conversation history
        self.conversation_history.append({
            "role": "user",
            "content": self.current_question,
            "timestamp": self.question_start_time
        })
        self.conversation_history.append({
            "role": "assistant",
            "content": response,
            "timestamp": time.time()
        })
        
        # Trim history if too long
        if len(self.conversation_history) > self.max_history * 2:
            self.conversation_history = self.conversation_history[-(self.max_history * 2):]
        
        # Send to TTS pipeline
        self.state = "answering"
        self.answer_start_time = time.time()
        
        node.send_output(
            "answer",
            pa.array([response]),
            metadata={
                "timestamp": time.time(),
                "question_num": self.questions_processed,
                "question": self.current_question
            }
        )
        
        self.log("INFO", "Answer sent to TTS, monitoring playback...")
        
    def on_buffer_status(self, node: Node, event: Dict[str, Any]):
        """Monitor audio buffer to detect when answer playback is complete."""
        if self.state != "answering":
            return
            
        buffer_percentage = event["value"][0].as_py()
        metadata = event.get("metadata", {})
        buffer_seconds = metadata.get("buffer_seconds", 0)
        
        # Rate limit buffer checks
        current_time = time.time()
        if current_time - self.last_buffer_check_time < 0.5:
            return
        self.last_buffer_check_time = current_time
        
        # Check if buffer is nearly empty (answer finished playing)
        if buffer_percentage < self.answer_complete_buffer_threshold:
            answer_duration = time.time() - self.answer_start_time
            self.log("INFO", f"Answer playback complete ({answer_duration:.1f}s, buffer: {buffer_percentage:.1f}%)")
            
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
                self.log("DEBUG", f"Answer playing... (buffer: {buffer_percentage:.1f}%, {buffer_seconds:.1f}s remaining)")
            
    def pause_speech_monitor(self, node: Node):
        """Send pause command to speech monitor."""
        self.log("INFO", "Pausing speech monitor")
        node.send_output(
            "speech_control",
            pa.array(["pause"]),
            metadata={"timestamp": time.time()}
        )
        
    def resume_speech_monitor(self, node: Node):
        """Send resume command to speech monitor."""
        self.log("INFO", "Resuming speech monitor")
        node.send_output(
            "speech_control",
            pa.array(["resume"]),
            metadata={"timestamp": time.time()}
        )
        
    def send_question_to_llm(self, node: Node, question: str):
        """Send the complete question to LLM with context."""
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
        
        self.log("INFO", f"Sending to LLM (with {len(self.conversation_history)} history items)")
        
        node.send_output(
            "question",
            pa.array([prompt]),
            metadata={
                "timestamp": time.time(),
                "question_num": self.questions_processed,
                "has_context": len(self.conversation_history) > 0,
                "raw_question": question
            }
        )
        
    def on_control(self, node: Node, event: Dict[str, Any]):
        """Handle control commands."""
        command = event["value"][0].as_py()
        
        if command == "reset":
            self.log("INFO", "Resetting conversation")
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
            self.log("INFO", f"Status: {json.dumps(status)}")
            
            node.send_output(
                "status",
                pa.array([json.dumps(status)]),
                metadata={"timestamp": time.time()}
            )


def main():
    """Main entry point for chat controller."""
    node = Node("chat-controller")
    controller = ChatController()
    
    print("=" * 60)
    print("VOICE CHATBOT - CHAT CONTROLLER")
    print("=" * 60)
    print(f"Min question length: {controller.min_question_length} chars")
    print(f"Max conversation history: {controller.max_history} exchanges")
    print(f"Answer complete threshold: {controller.answer_complete_buffer_threshold}%")
    print("=" * 60)
    print("Ready for conversation...")
    print()
    
    for event in node:
        try:
            if event["type"] == "INPUT":
                input_id = event["id"]
                
                if input_id == "speech_started":
                    controller.on_speech_started(node, event)
                    
                elif input_id == "speech_ended":
                    controller.on_speech_ended(node, event)
                    
                elif input_id == "transcription":
                    controller.on_transcription(node, event)
                    
                elif input_id == "llm_response":
                    controller.on_llm_response(node, event)
                    
                elif input_id == "buffer_status":
                    controller.on_buffer_status(node, event)
                    
                elif input_id == "control":
                    controller.on_control(node, event)
                    
            elif event["type"] == "STOP":
                break
                
        except Exception as e:
            controller.log("ERROR", f"Error processing event: {e}")
            
    print(f"\n[Chat Controller] Stopped after {controller.questions_processed} questions")
    print("=" * 60)


if __name__ == "__main__":
    main()