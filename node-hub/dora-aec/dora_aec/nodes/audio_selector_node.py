#!/usr/bin/env python3
"""
Audio Selector Node - Switches between audio sources
"""

import time
import pyarrow as pa
from dora import Node


class AudioSelector:
    def __init__(self):
        self.use_primary = True
        self.last_primary_time = time.time()
        self.timeout = 1.0  # Switch to fallback after 1s of no primary
    
    def select(self, primary_audio=None, fallback_audio=None):
        """Select best available audio source"""
        current_time = time.time()
        
        if primary_audio is not None:
            self.last_primary_time = current_time
            self.use_primary = True
            return primary_audio
        
        # Check if primary timed out
        if current_time - self.last_primary_time > self.timeout:
            self.use_primary = False
        
        if not self.use_primary and fallback_audio is not None:
            return fallback_audio
        
        return None


def main():
    node = Node()
    selector = AudioSelector()
    
    primary_buffer = None
    fallback_buffer = None
    
    while True:
        event = node.next(timeout=0.01)
        
        if event and event["type"] == "INPUT":
            if event["id"] == "primary":
                primary_buffer = event["value"]
            elif event["id"] == "fallback":
                fallback_buffer = event["value"]
            elif event["id"] == "control":
                cmd = event["value"][0].as_py()
                if cmd == "use_primary":
                    selector.use_primary = True
                elif cmd == "use_fallback":
                    selector.use_primary = False
        
        # Select and forward audio
        selected = selector.select(primary_buffer, fallback_buffer)
        if selected is not None:
            node.send_output("selected_audio", selected)
            # Clear buffers after sending
            if selected == primary_buffer:
                primary_buffer = None
            else:
                fallback_buffer = None


if __name__ == "__main__":
    main()