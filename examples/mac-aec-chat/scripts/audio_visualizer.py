#!/usr/bin/env python3
"""
Real-time audio visualization for debugging
Shows waveform and VAD status
"""

import numpy as np
import pyarrow as pa
from dora import Node
import time
import threading
from collections import deque
import matplotlib.pyplot as plt
import matplotlib.animation as animation


class AudioVisualizer:
    def __init__(self):
        self.node = Node()
        
        # Audio buffer for visualization
        self.audio_buffer = deque(maxlen=8000)  # ~0.5s at 16kHz
        self.vad_status = False
        self.sample_rate = 16000
        
        # Setup matplotlib
        plt.ion()  # Interactive mode
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(12, 8))
        self.fig.suptitle('Real-time Audio Monitor')
        
        # Waveform plot
        self.ax1.set_title('Audio Waveform (500ms window)')
        self.ax1.set_ylabel('Amplitude')
        self.ax1.set_ylim(-1, 1)
        self.line1, = self.ax1.plot([], [], 'b-', linewidth=0.5)
        
        # VAD status plot
        self.ax2.set_title('Voice Activity Detection')
        self.ax2.set_ylabel('VAD Active')
        self.ax2.set_xlabel('Time')
        self.ax2.set_ylim(-0.1, 1.1)
        self.vad_line, = self.ax2.plot([], [], 'g-', linewidth=2)
        
        # VAD history
        self.vad_history = deque(maxlen=200)
        self.time_history = deque(maxlen=200)
        self.start_time = time.time()
        
    def update_display(self):
        """Update the visualization"""
        try:
            if len(self.audio_buffer) > 0:
                # Update waveform
                audio_data = np.array(list(self.audio_buffer))
                time_axis = np.linspace(0, len(audio_data)/self.sample_rate, len(audio_data))
                
                self.line1.set_data(time_axis, audio_data)
                self.ax1.set_xlim(0, len(audio_data)/self.sample_rate)
                
                # Update VAD history
                current_time = time.time() - self.start_time
                self.time_history.append(current_time)
                self.vad_history.append(1.0 if self.vad_status else 0.0)
                
                if len(self.time_history) > 1:
                    self.vad_line.set_data(list(self.time_history), list(self.vad_history))
                    self.ax2.set_xlim(max(0, current_time - 30), current_time + 1)
                
                # Refresh display
                self.fig.canvas.draw()
                self.fig.canvas.flush_events()
                
        except Exception as e:
            print(f"Display update error: {e}")
    
    def run(self):
        """Main processing loop"""
        print("Starting audio visualizer...")
        print("Press Ctrl+C to stop")
        
        last_display_update = time.time()
        display_interval = 1.0 / 30.0  # 30 FPS
        
        try:
            while True:
                event = self.node.next(timeout=0.01)
                
                if event and event["type"] == "INPUT":
                    if event["id"] == "audio":
                        # Add audio data to buffer
                        audio_data = event["value"].to_numpy()
                        self.audio_buffer.extend(audio_data)
                        
                    elif event["id"] == "vad":
                        # Update VAD status
                        self.vad_status = event["value"][0].as_py()
                
                # Update display at fixed rate
                current_time = time.time()
                if current_time - last_display_update >= display_interval:
                    self.update_display()
                    last_display_update = current_time
                    
                    # Send visualization status
                    status = {
                        "buffer_size": len(self.audio_buffer),
                        "vad_active": self.vad_status,
                        "display_fps": 1.0 / display_interval
                    }
                    self.node.send_output("visualization", pa.array([str(status)]))
                    
        except KeyboardInterrupt:
            print("\nStopping visualizer...")
        except Exception as e:
            print(f"Visualizer error: {e}")
        finally:
            plt.close('all')


if __name__ == "__main__":
    visualizer = AudioVisualizer()
    visualizer.run()