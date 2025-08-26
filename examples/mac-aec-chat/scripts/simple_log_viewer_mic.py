#!/usr/bin/env python3
"""
Simple log viewer operator for testing microphone and speech-monitor
"""

import time
import json
from dora import DoraStatus
import pyarrow as pa


class Operator:
    """Simple operator to display logs and events"""
    
    def __init__(self):
        self.start_time = time.time()
        self.event_count = 0
        self.speech_events = []
        self.audio_received = False
        
        print("=" * 60)
        print("MICROPHONE + SPEECH-MONITOR TEST")
        print("=" * 60)
        print("Testing standard microphone input with speech-monitor...")
        print("Speak to test speech detection!")
        print()
    
    def on_event(self, dora_event, send_output):
        """Handle incoming events"""
        self.event_count += 1
        elapsed = time.time() - self.start_time
        
        event_type = dora_event["type"]
        
        if event_type == "INPUT":
            input_id = dora_event["id"]
            
            if input_id == "speech_log":
                # Parse and display speech monitor log
                try:
                    log_data = dora_event["value"][0].as_py()
                    if isinstance(log_data, str):
                        try:
                            log_info = json.loads(log_data)
                            message = log_info.get("message", log_data)
                        except:
                            message = log_data
                    else:
                        message = str(log_data)
                    
                    # Only show important messages
                    if any(key in message.lower() for key in ["audio chunk", "no audio", "empty", "speaking", "silence"]):
                        print(f"[{elapsed:6.1f}s] SPEECH-MONITOR: {message}")
                    
                    # Check for audio flow
                    if "audio chunk" in message.lower() and not self.audio_received:
                        print("  >>> CONFIRMED: Audio is flowing from microphone to speech-monitor!")
                        self.audio_received = True
                        
                except Exception as e:
                    pass  # Ignore parse errors
            
            elif input_id == "speech_started":
                print(f"[{elapsed:6.1f}s] *** SPEECH STARTED ***")
                self.speech_events.append(("started", elapsed))
                
            elif input_id == "speech_ended":
                print(f"[{elapsed:6.1f}s] *** SPEECH ENDED ***")
                self.speech_events.append(("ended", elapsed))
                
            elif input_id == "is_speaking":
                try:
                    is_speaking = dora_event["value"][0].as_py()
                    if is_speaking:
                        print(f"[{elapsed:6.1f}s] >>> USER IS SPEAKING")
                except:
                    pass
            
            # Send periodic status
            if self.event_count % 20 == 0:
                status = f"Events: {self.event_count}, Speech events: {len(self.speech_events)}, Audio: {'YES' if self.audio_received else 'NO'}"
                send_output("display", pa.array([status]), dora_event["metadata"])
                
        elif event_type == "STOP":
            print()
            print("=" * 60)
            print("TEST SUMMARY")
            print("=" * 60)
            print(f"Total events: {self.event_count}")
            print(f"Audio received: {'YES' if self.audio_received else 'NO'}")
            print(f"Speech events: {len(self.speech_events)}")
            if self.speech_events:
                print("Speech timeline:")
                for event_type, event_time in self.speech_events:
                    print(f"  {event_time:6.1f}s: {event_type}")
            print("=" * 60)
            return DoraStatus.STOP
        
        return DoraStatus.CONTINUE