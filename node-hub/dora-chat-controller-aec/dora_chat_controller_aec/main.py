#!/usr/bin/env python3
"""
Simplified Chat Controller for AEC Voice Chatbot

Orchestrates conversation flow without pause/resume logic.
AEC handles echo cancellation automatically.
"""

import time
from typing import Optional, List, Dict, Any

import pyarrow as pa
from dora import Node


class Config:
    """Configuration for Chat Controller"""
    NODE_TIMEOUT_MS = 1000  # Timeout for node.next() in milliseconds
    MAX_HISTORY_LENGTH = 10  # Maximum conversation history to maintain


class ChatControllerAEC:
    """
    Simplified chat controller for AEC-enabled voice chatbot
    
    No pause/resume needed - AEC handles echo cancellation automatically.
    """
    
    def __init__(self, node: Node):
        self.node = node
        self.config = Config()
        
        # State
        self.current_transcription = ""
        self.conversation_history: List[Dict[str, str]] = []
        self.transcription_count = 0
        
        # Load configuration
        self._load_config()
        
        self._log_info("Chat Controller AEC initialized")
    
    def _load_config(self):
        """Load configuration from environment variables"""
        import os
        
        self.config.NODE_TIMEOUT_MS = int(os.getenv("NODE_TIMEOUT_MS", str(self.config.NODE_TIMEOUT_MS)))
        self.config.MAX_HISTORY_LENGTH = int(os.getenv("MAX_HISTORY_LENGTH", str(self.config.MAX_HISTORY_LENGTH)))
    
    def on_transcription(self, event: Dict[str, Any]):
        """
        Handle transcription from ASR
        
        With AEC, we don't need to check if we should process it.
        The user can speak at any time.
        """
        transcription = event["value"][0].as_py()
        
        if not transcription or not transcription.strip():
            return
        
        self.current_transcription = transcription
        self.transcription_count += 1
        
        self._log_debug(f"Transcription #{self.transcription_count}: {transcription}")
    
    def on_question_ended(self, event: Dict[str, Any]):
        """
        Handle question_ended signal from Speech Monitor
        
        This indicates a longer silence after speech, suggesting
        the user has finished their question.
        """
        if not self.current_transcription:
            self._log_debug("Question ended but no transcription to send")
            return
        
        # Send the question to the LLM
        self._send_question_to_llm(self.current_transcription)
        
        # Clear current transcription
        self.current_transcription = ""
    
    def on_segments(self, event: Dict[str, Any]):
        """
        Handle segments from text segmenter (LLM response)
        
        With AEC, we just log the response. No need to manage
        playback state since echo is automatically cancelled.
        """
        segments = event["value"].to_pylist()
        
        if segments:
            response = " ".join(segments)
            self._log_debug(f"LLM response received: {response[:100]}...")
    
    def _send_question_to_llm(self, question: str):
        """Send question to the LLM"""
        # Add to conversation history
        self.conversation_history.append({"role": "user", "content": question})
        
        # Trim history if too long
        if len(self.conversation_history) > self.config.MAX_HISTORY_LENGTH:
            self.conversation_history = self.conversation_history[-self.config.MAX_HISTORY_LENGTH:]
        
        # Prepare the prompt with history
        prompt = self._prepare_prompt_with_history(question)
        
        # Send to LLM
        self.node.send_output("question", pa.array([prompt]))
        
        self._log_info(f"Question sent to LLM: {question}")
    
    def _prepare_prompt_with_history(self, current_question: str) -> str:
        """
        Prepare prompt with conversation history
        
        Returns the current question with relevant context from history.
        """
        if not self.conversation_history[:-1]:  # No previous history
            return current_question
        
        # Include last few exchanges for context
        context_exchanges = self.conversation_history[-4:-1] if len(self.conversation_history) > 1 else []
        
        if context_exchanges:
            context = "\n".join([
                f"{'User' if item['role'] == 'user' else 'Assistant'}: {item['content']}"
                for item in context_exchanges
            ])
            return f"Previous context:\n{context}\n\nCurrent question: {current_question}"
        
        return current_question
    
    def _log_info(self, message: str):
        """Send info log message"""
        log_msg = f"[INFO ] Chat Controller AEC: {message}"
        self.node.send_output("log", pa.array([log_msg]))
        print(f"{time.strftime('%H:%M:%S.%f')[:-3]} {log_msg}")
    
    def _log_debug(self, message: str):
        """Send debug log message"""
        log_msg = f"[DEBUG] Chat Controller AEC: {message}"
        self.node.send_output("log", pa.array([log_msg]))
        print(f"{time.strftime('%H:%M:%S.%f')[:-3]} {log_msg}")
    
    def _log_error(self, message: str):
        """Send error log message"""
        log_msg = f"[ERROR] Chat Controller AEC: {message}"
        self.node.send_output("log", pa.array([log_msg]))
        print(f"{time.strftime('%H:%M:%S.%f')[:-3]} {log_msg}")


def main():
    """Main entry point for Chat Controller AEC node"""
    node = Node()
    controller = ChatControllerAEC(node)
    
    timeout_seconds = controller.config.NODE_TIMEOUT_MS / 1000.0
    
    try:
        while True:
            event = node.next(timeout=timeout_seconds)
            
            if event is None:
                continue
            
            if event["type"] == "INPUT":
                input_id = event.get("id", "")
                
                if input_id == "transcription":
                    controller.on_transcription(event)
                elif input_id == "question_ended":
                    controller.on_question_ended(event)
                elif input_id == "segments":
                    controller.on_segments(event)
                    
    except KeyboardInterrupt:
        controller._log_info("Received interrupt signal")
    except Exception as e:
        controller._log_error(f"Unexpected error: {e}")
        raise


if __name__ == "__main__":
    main()