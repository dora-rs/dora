#!/usr/bin/env python3
"""
Dynamic Log Display Node for Voice Chatbot
Collects and displays logs from all static nodes in a formatted way.
"""

import json
import time
from datetime import datetime
from collections import deque
from dora import Node, DoraStatus
import pyarrow as pa


class LogDisplay:
    """Manages log display with formatting and filtering."""
    
    def __init__(self, max_logs=100):
        self.max_logs = max_logs
        self.log_buffer = deque(maxlen=max_logs)
        self.node_colors = {
            "speech-monitor": "\033[36m",      # Cyan
            "asr": "\033[35m",                 # Magenta
            "chat-controller": "\033[32m",     # Green
            "qwen3-llm": "\033[33m",          # Yellow
            "text-segmenter": "\033[34m",     # Blue
            "conversation-controller": "\033[95m",  # Light Magenta
            "primespeech": "\033[94m",        # Light Blue
            "audio-player": "\033[96m",       # Light Cyan
            "default": "\033[37m"             # White
        }
        self.reset_color = "\033[0m"
        self.last_display_time = 0
        self.display_interval = 0.1  # Update display every 100ms
        
    def format_timestamp(self, timestamp):
        """Format timestamp for display."""
        dt = datetime.fromtimestamp(timestamp)
        return dt.strftime("%H:%M:%S.%f")[:-3]  # HH:MM:SS.mmm
        
    def get_node_color(self, node_name):
        """Get color code for a node."""
        return self.node_colors.get(node_name, self.node_colors["default"])
        
    def format_log_entry(self, log_data):
        """Format a single log entry for display."""
        try:
            # Parse log data if it's a string
            if isinstance(log_data, str):
                try:
                    log_info = json.loads(log_data)
                except json.JSONDecodeError:
                    # Handle non-JSON log format (plain string)
                    # Try to extract parts from formatted string like "[LEVEL] [Node] Message"
                    import re
                    match = re.match(r'\[(\w+)\]\s*\[([^\]]+)\]\s*(.*)', log_data)
                    if match:
                        log_info = {
                            "level": match.group(1),
                            "node": match.group(2),
                            "message": match.group(3),
                            "timestamp": time.time()
                        }
                    else:
                        # Fallback for unformatted logs
                        log_info = {
                            "node": "unknown",
                            "level": "INFO",
                            "message": log_data,
                            "timestamp": time.time()
                        }
            else:
                log_info = log_data
                
            node = log_info.get("node", "unknown")
            level = log_info.get("level", "INFO")
            message = log_info.get("message", "")
            timestamp = log_info.get("timestamp", time.time())
            
            # Color coding for log levels
            level_colors = {
                "ERROR": "\033[91m",    # Red
                "WARNING": "\033[93m",  # Yellow
                "INFO": "\033[92m",     # Green
                "DEBUG": "\033[90m"     # Gray
            }
            
            level_color = level_colors.get(level, "")
            node_color = self.get_node_color(node)
            
            # Format the log line
            time_str = self.format_timestamp(timestamp)
            
            # Clean up the message - remove redundant level prefix if present
            if message.startswith(f"[{level}]"):
                message = message[len(f"[{level}]"):].strip()
            
            # Special formatting for certain message types
            if "Question complete" in message or "Answer:" in message:
                message = f"\033[1m{message}\033[0m"  # Bold
            
            formatted = (
                f"{time_str} "
                f"{level_color}[{level:7}]{self.reset_color} "
                f"{node_color}{node:20}{self.reset_color} "
                f"{message}"
            )
            
            return formatted
            
        except Exception as e:
            return f"Error formatting log: {e} - Raw: {log_data}"
    
    def add_log(self, log_data):
        """Add a log entry to the buffer."""
        self.log_buffer.append(log_data)
        
    def should_display(self):
        """Check if it's time to update the display."""
        current_time = time.time()
        if current_time - self.last_display_time >= self.display_interval:
            self.last_display_time = current_time
            return True
        return False
        
    def display(self):
        """Display recent logs with formatting."""
        if not self.should_display():
            return
            
        # Clear screen and move cursor to top
        print("\033[2J\033[H", end="")
        
        # Header
        print("=" * 100)
        print(f"{self.get_node_color('chat-controller')}🎙️  VOICE CHATBOT LOG DISPLAY{self.reset_color}".center(110))
        print("=" * 100)
        print()
        
        # Display recent logs
        if self.log_buffer:
            print(f"Recent Logs (showing last {len(self.log_buffer)} entries):")
            print("-" * 100)
            
            for log_data in self.log_buffer:
                print(self.format_log_entry(log_data))
            
        else:
            print("Waiting for logs...")
        
        # Footer with stats
        print()
        print("-" * 100)
        print(f"Total logs: {len(self.log_buffer)} | Press Ctrl+C to exit")


def main():
    """Main entry point for log display node."""
    # Initialize as dynamic node with ID
    node = Node("log-display")
    display = LogDisplay(max_logs=50)  # Keep last 50 logs
    
    # Initial display
    print("\033[2J\033[H")  # Clear screen
    print("=" * 100)
    print("🎙️  VOICE CHATBOT LOG DISPLAY - Starting...".center(100))
    print("=" * 100)
    print("\nWaiting for logs from nodes...")
    
    # Process events
    for event in node:
        if event["type"] == "INPUT":
            input_id = event["id"]
            
            # Handle log inputs from various nodes
            if input_id.endswith("_log") or input_id == "log":
                try:
                    # Get log data
                    log_value = event["value"][0].as_py()
                    
                    # Add to display buffer
                    display.add_log(log_value)
                    
                    # Update display
                    display.display()
                    
                except Exception as e:
                    print(f"Error processing log: {e}")
                    
        elif event["type"] == "STOP":
            print("\n" + "=" * 100)
            print("Log Display stopped.".center(100))
            print("=" * 100)
            break
            
        # Periodic display update even without new logs
        display.display()
    
    return DoraStatus.STOP


if __name__ == "__main__":
    main()