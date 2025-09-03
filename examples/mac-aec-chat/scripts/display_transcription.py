#!/usr/bin/env python3
"""
Display real-time transcription with partial results
"""

import pyarrow as pa
from dora import Node
import time


class TranscriptionDisplay:
    def __init__(self):
        self.node = Node()
        
        # Display state
        self.current_text = ""
        self.partial_text = ""
        self.last_update = time.time()
        
        print("Starting transcription display...")
        print("=" * 60)
    
    def update_display(self):
        """Update the transcription display"""
        # Clear screen and show header
        print("\033[2J\033[H")
        print("REAL-TIME TRANSCRIPTION")
        print("=" * 60)
        
        # Show completed text
        if self.current_text:
            print("COMPLETED:")
            print(f"  {self.current_text}")
            print()
        
        # Show partial text (in progress)
        if self.partial_text:
            print("PARTIAL:")
            print(f"  \033[93m{self.partial_text}\033[0m")  # Yellow text
            print()
        
        # Show status
        print(f"Last update: {time.strftime('%H:%M:%S')}")
        print("Press Ctrl+C to stop")
        print("-" * 60)
    
    def run(self):
        """Main display loop"""
        try:
            while True:
                event = self.node.next(timeout=0.1)
                
                if event and event["type"] == "INPUT":
                    updated = False
                    
                    if event["id"] == "text":
                        # Final transcription text
                        new_text = event["value"][0].as_py().strip()
                        if new_text and new_text != self.current_text:
                            self.current_text = new_text
                            self.partial_text = ""  # Clear partial when we get final
                            updated = True
                            
                            # Send display status
                            self.node.send_output("display", pa.array([f"Final: {new_text}"]))
                    
                    elif event["id"] == "partial":
                        # Partial transcription (work in progress)
                        new_partial = event["value"][0].as_py().strip()
                        if new_partial != self.partial_text:
                            self.partial_text = new_partial
                            updated = True
                            
                            # Send display status
                            self.node.send_output("display", pa.array([f"Partial: {new_partial}"]))
                    
                    if updated:
                        self.update_display()
                        self.last_update = time.time()
                
                # Periodic refresh (in case display gets corrupted)
                if time.time() - self.last_update > 5.0:
                    self.update_display()
                    self.last_update = time.time()
                    
        except KeyboardInterrupt:
            print("\n\nTranscription display stopped.")
            if self.current_text:
                print(f"Final transcription: {self.current_text}")
        except Exception as e:
            print(f"Display error: {e}")


if __name__ == "__main__":
    display = TranscriptionDisplay()
    display.run()