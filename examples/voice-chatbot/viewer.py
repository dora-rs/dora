#!/usr/bin/env python3
"""
Simple Viewer Node for Voice Assistant Pipeline
Displays transcriptions, LLM outputs, and system events
"""

import sys
import json
import time
from datetime import datetime
import pyarrow as pa
from dora import Node


class Colors:
    """ANSI color codes for terminal output"""
    HEADER = '\033[95m'
    BLUE = '\033[94m'
    CYAN = '\033[96m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


def format_timestamp():
    """Format current time for display"""
    return datetime.now().strftime("%H:%M:%S")


def print_event(event_type, content, color=Colors.ENDC):
    """Print formatted event to console"""
    timestamp = format_timestamp()
    print(f"{Colors.BOLD}[{timestamp}]{Colors.ENDC} {color}{event_type}:{Colors.ENDC} {content}")


def main():
    """Main viewer loop"""
    node = Node("viewer")
    
    print("\n" + "="*60)
    print(f"{Colors.BOLD}🔍 Voice Assistant Pipeline Viewer{Colors.ENDC}")
    print("="*60)
    print("Monitoring pipeline events...\n")
    
    # Track conversation state
    last_transcription = ""
    last_response = ""
    
    for event in node:
        if event["type"] == "INPUT":
            input_id = event["id"]
            
            try:
                # Handle different input types
                if input_id == "transcription":
                    # ASR transcription
                    text = event["value"][0].as_py()
                    if text and text != last_transcription:
                        print_event("🎤 USER", text, Colors.CYAN)
                        last_transcription = text
                        
                elif input_id == "llm_output":
                    # LLM response
                    text = event["value"][0].as_py()
                    if text and text != last_response:
                        # Handle streaming - might get chunks
                        if len(text) > 200:
                            # Full response
                            print_event("🤖 ASSISTANT", text[:200] + "...", Colors.GREEN)
                        else:
                            print_event("🤖 ASSISTANT", text, Colors.GREEN)
                        last_response = text
                        
                elif input_id == "segment":
                    # Text segment being sent to TTS
                    text = event["value"][0].as_py()
                    if text:
                        print_event("🔊 TTS", f"Speaking: '{text[:50]}...'", Colors.YELLOW)
                        
                elif input_id == "speech_started":
                    # User started speaking
                    print_event("🎙️ SPEECH", "User started speaking", Colors.BLUE)
                    
                elif input_id == "speech_ended":
                    # User stopped speaking
                    print_event("🎙️ SPEECH", "User stopped speaking", Colors.BLUE)
                    
                elif input_id == "log":
                    # Log messages from nodes
                    try:
                        log_data = json.loads(event["value"][0].as_py())
                        level = log_data.get("level", "INFO")
                        message = log_data.get("message", "")
                        
                        if level == "ERROR":
                            print_event("⚠️ ERROR", message, Colors.RED)
                        elif level == "WARNING":
                            print_event("⚠️ WARNING", message, Colors.YELLOW)
                        elif level == "DEBUG":
                            # Skip debug messages in normal view
                            pass
                        else:
                            # Info messages
                            if "Ready" in message or "ready" in message:
                                print_event("✅ SYSTEM", message, Colors.GREEN)
                                
                    except:
                        pass
                        
            except Exception as e:
                print_event("❌ ERROR", f"Error processing {input_id}: {e}", Colors.RED)
                
        elif event["type"] == "STOP":
            print_event("🛑 SYSTEM", "Pipeline stopped", Colors.YELLOW)
            break
            
    print("\n" + "="*60)
    print(f"{Colors.BOLD}Viewer stopped{Colors.ENDC}")
    print("="*60 + "\n")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nViewer interrupted by user")
        sys.exit(0)
    except Exception as e:
        print(f"\n{Colors.RED}Fatal error: {e}{Colors.ENDC}")
        sys.exit(1)