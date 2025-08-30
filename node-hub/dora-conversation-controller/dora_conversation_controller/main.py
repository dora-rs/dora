#!/usr/bin/env python3
"""
Dora Conversation Controller Node - Main Module.
Controls conversation flow based on audio buffer status to prevent overrun.
"""

import os
import time
import json
import pyarrow as pa
from dora import Node
from typing import Optional, Dict, Any


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


class ConversationController:
    """
    Manages conversation flow with hysteresis-based backpressure control.
    """
    
    def __init__(self, config: Dict[str, Any]):
        """
        Initialize controller with configuration.
        
        Args:
            config: Configuration dictionary with thresholds and parameters
        """
        # Load thresholds from config or environment
        self.pause_threshold = float(config.get("PAUSE_THRESHOLD", 
                                                os.getenv("PAUSE_THRESHOLD", "70")))
        self.resume_threshold = float(config.get("RESUME_THRESHOLD", 
                                                 os.getenv("RESUME_THRESHOLD", "40")))
        self.min_control_interval = float(config.get("MIN_CONTROL_INTERVAL", 
                                                     os.getenv("MIN_CONTROL_INTERVAL", "1.0")))
        
        # Validate thresholds
        if self.resume_threshold >= self.pause_threshold:
            raise ValueError(f"Resume threshold ({self.resume_threshold}%) must be less than "
                           f"pause threshold ({self.pause_threshold}%)")
        
        # State management
        self.generation_paused = False
        self.last_buffer_percentage = 0.0
        self.last_control_change = 0  # Set to 0 so first control change is allowed
        
        # Statistics
        self.pause_count = 0
        self.resume_count = 0
        self.total_decisions = 0
        self.buffer_readings = 0
        self.start_time = time.time()
        
        # Session tracking
        self.active_sessions = {}  # Track multiple sessions if needed
        
        # Configuration will be logged in main()
    
    def should_change_state(self, buffer_percentage: float) -> bool:
        """
        Determine if flow control state should change.
        Implements hysteresis to prevent oscillation.
        
        Args:
            buffer_percentage: Current buffer fullness percentage
            
        Returns:
            True if state should change, False otherwise
        """
        current_time = time.time()
        
        # Prevent rapid state changes
        if current_time - self.last_control_change < self.min_control_interval:
            return False
        
        # Check pause condition
        if not self.generation_paused and buffer_percentage > self.pause_threshold:
            return True
        
        # Check resume condition
        if self.generation_paused and buffer_percentage < self.resume_threshold:
            return True
        
        return False
    
    def process_buffer_status(self, buffer_percentage: float, 
                            metadata: Optional[Dict[str, Any]] = None) -> Optional[str]:
        """
        Process buffer status update and determine control action.
        
        Args:
            buffer_percentage: Current buffer fullness percentage
            metadata: Optional metadata from buffer status
            
        Returns:
            Control signal ("pause" or "continue") for every buffer status
        """
        self.buffer_readings += 1
        self.last_buffer_percentage = buffer_percentage
        
        current_time = time.time()
        elapsed = current_time - self.start_time
        
        # Check if we should pause (buffer too full)
        if not self.generation_paused and buffer_percentage > self.pause_threshold:
            # Pause generation - buffer is getting full
            self.generation_paused = True
            self.pause_count += 1
            self.last_control_change = current_time
            self.total_decisions += 1
            
            # Log will be sent from main function
            self.pause_message = f"PAUSING generation (buffer: {buffer_percentage:.1f}% > {self.pause_threshold}%)"
            return "pause"
        
        # Check if we should resume (was paused but buffer drained enough)
        elif self.generation_paused and buffer_percentage < self.resume_threshold:
            # Resume generation - buffer has drained enough
            self.generation_paused = False
            self.resume_count += 1
            self.last_control_change = current_time
            self.total_decisions += 1
            
            # Log will be sent from main function
            self.resume_message = f"RESUMING generation (buffer: {buffer_percentage:.1f}% < {self.resume_threshold}%)"
            return "continue"
        
        # Otherwise, send continue if not paused
        elif not self.generation_paused:
            # Not paused, buffer has space - continue sending
            return "continue"
        
        # If paused and buffer still above resume threshold, don't send anything
        return None
    
    def get_status(self) -> Dict[str, Any]:
        """
        Get current controller status and statistics.
        
        Returns:
            Dictionary with status information
        """
        current_time = time.time()
        elapsed = current_time - self.start_time
        
        return {
            "state": "paused" if self.generation_paused else "flowing",
            "buffer_percentage": self.last_buffer_percentage,
            "thresholds": {
                "pause": self.pause_threshold,
                "resume": self.resume_threshold
            },
            "statistics": {
                "runtime": elapsed,
                "buffer_readings": self.buffer_readings,
                "pause_count": self.pause_count,
                "resume_count": self.resume_count,
                "total_decisions": self.total_decisions
            }
        }


def main():
    """
    Main entry point for the conversation controller node.
    """
    # Initialize node (for static node, no ID needed)
    node = Node()
    
    # Configuration from environment or defaults
    config = {
        "PAUSE_THRESHOLD": os.getenv("PAUSE_THRESHOLD", "70"),
        "RESUME_THRESHOLD": os.getenv("RESUME_THRESHOLD", "40"),
        "MIN_CONTROL_INTERVAL": os.getenv("MIN_CONTROL_INTERVAL", "1.0"),
        "STATUS_INTERVAL": float(os.getenv("STATUS_INTERVAL", "5.0")),
        "LOG_LEVEL": os.getenv("LOG_LEVEL", "INFO")
    }
    
    # Initialize controller
    controller = ConversationController(config)
    
    # Display startup information
    print("╔══════════════════════════════════════════════════════════╗")
    print("║         DORA CONVERSATION CONTROLLER NODE                 ║")
    print("╠══════════════════════════════════════════════════════════╣")
    print("║ Backpressure control for streaming TTS                    ║")
    print(f"║ Pause at >{controller.pause_threshold:3.0f}% | Resume at <{controller.resume_threshold:3.0f}%                ║")
    print("╚══════════════════════════════════════════════════════════╝")
    print()
    
    # Timing for periodic status
    last_status_time = time.time()
    status_interval = config["STATUS_INTERVAL"]
    
    # Main event loop
    for event in node:
        if event["type"] == "INPUT":
            
            # Process buffer status input
            if event["id"] == "buffer_status":
                try:
                    # Extract buffer percentage
                    raw_value = event["value"]
                    if raw_value is None or len(raw_value) == 0:
                        continue
                    
                    buffer_percentage = raw_value[0].as_py()
                    metadata = event.get("metadata", {})
                    
                    # Process and get control decision
                    control_signal = controller.process_buffer_status(buffer_percentage, metadata)
                    
                    # Send control signal if state changed
                    if control_signal:
                        # Log the control decision
                        elapsed = time.time() - controller.start_time
                        if hasattr(controller, 'pause_message'):
                            send_log(node, "INFO", f"[{elapsed:7.1f}s] {controller.pause_message}")
                        elif hasattr(controller, 'resume_message'):
                            send_log(node, "INFO", f"[{elapsed:7.1f}s] {controller.resume_message}")
                        
                        # Extract session info if available
                        session_id = metadata.get("session_id", "default")
                        
                        # Send segment control output
                        node.send_output(
                            "segment_control",
                            pa.array([control_signal]),
                            metadata={
                                "control": control_signal,
                                "buffer_percentage": buffer_percentage,
                                "session_id": session_id,
                                "timestamp": time.time(),
                                "reason": f"buffer_{control_signal}",
                                "pause_threshold": controller.pause_threshold,
                                "resume_threshold": controller.resume_threshold
                            }
                        )
                        
                        # Log if debug level
                        if config["LOG_LEVEL"] == "DEBUG":
                            send_log(node, "DEBUG", f"Sent control signal: {control_signal}")
                
                except Exception as e:
                    send_log(node, "ERROR", f"Failed to process buffer status: {e}")
                    continue
        
        elif event["type"] == "STOP":
            send_log(node, "INFO", "Received STOP signal")
            break
        
        # Periodic status display
        current_time = time.time()
        if current_time - last_status_time >= status_interval:
            status = controller.get_status()
            elapsed = status["statistics"]["runtime"]
            
            send_log(node, "INFO", f"[{elapsed:7.1f}s] Controller Status: {status['state'].upper()}")
            send_log(node, "INFO", f"  Buffer: {status['buffer_percentage']:.1f}%")
            send_log(node, "INFO", f"  Readings: {status['statistics']['buffer_readings']}")
            send_log(node, "INFO", f"  Control changes: {status['statistics']['pause_count']} pauses, "
                     f"{status['statistics']['resume_count']} resumes")
            
            # Send status output if configured
            try:
                # Convert status to JSON string for sending
                # Don't include complex nested dicts in metadata
                node.send_output(
                    "status",
                    pa.array([json.dumps(status)]),
                    metadata={
                        "state": status["state"],
                        "buffer_percentage": status["buffer_percentage"],
                        "pause_count": status["statistics"]["pause_count"],
                        "resume_count": status["statistics"]["resume_count"],
                        "runtime": status["statistics"]["runtime"]
                    }
                )
            except Exception as e:
                if config["LOG_LEVEL"] == "DEBUG":
                    send_log(node, "DEBUG", f"Could not send status: {e}")
                pass  # Status output is optional
            
            last_status_time = current_time
    
    # Final statistics
    final_status = controller.get_status()
    send_log(node, "INFO", "=" * 60)
    send_log(node, "INFO", "Conversation Controller Final Statistics")
    send_log(node, "INFO", "=" * 60)
    send_log(node, "INFO", f"Total runtime: {final_status['statistics']['runtime']:.1f}s")
    send_log(node, "INFO", f"Buffer readings: {final_status['statistics']['buffer_readings']}")
    send_log(node, "INFO", f"Control decisions: {final_status['statistics']['total_decisions']}")
    send_log(node, "INFO", f"Pause count: {final_status['statistics']['pause_count']}")
    send_log(node, "INFO", f"Resume count: {final_status['statistics']['resume_count']}")
    send_log(node, "INFO", "=" * 60)


if __name__ == "__main__":
    main()