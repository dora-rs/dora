#!/usr/bin/env python3
"""
Real-time sound level visualizer for AEC output.
Displays audio levels, VAD status, and spectral information.
"""

import sys
import time
import numpy as np
import pyarrow as pa
from dora import Node
from collections import deque
import threading

# Audio configuration
SAMPLE_RATE = 16000  # 16kHz from AEC
CHANNELS = 1         # Mono
DTYPE = np.float32

# ANSI colors and styles
GREEN = "\033[32m"
YELLOW = "\033[33m"
CYAN = "\033[36m"
MAGENTA = "\033[35m"
BLUE = "\033[34m"
RED = "\033[31m"
WHITE = "\033[37m"
RESET = "\033[0m"
BOLD = "\033[1m"
DIM = "\033[2m"
CLEAR_LINE = "\033[K"
CURSOR_UP = "\033[A"

class SoundVisualizer:
    def __init__(self):
        print(f"{CYAN}{BOLD}AEC Frequency Spectrum Monitor{RESET}")
        print(f"{CYAN}{'='*60}{RESET}")
        
        # Connect as dynamic node
        self.node = Node("sound-visualizer")
        print(f"{GREEN}✓ Connected as 'sound-visualizer' node{RESET}")
        
        # Statistics
        self.chunks_received = 0
        self.start_time = time.time()
        self.vad_active = False
        self.vad_history = deque(maxlen=50)  # VAD history for visualization
        
        # Audio level history for averaging
        self.rms_history = deque(maxlen=10)
        self.peak_history = deque(maxlen=10)
        
        # Frequency analysis
        self.freq_bins = 8  # Number of frequency bands to display
        self.freq_levels = np.zeros(self.freq_bins)
        
        # Display control
        self.last_display_time = 0
        self.display_interval = 0.05  # Update display every 50ms
        
    def calculate_frequency_bands(self, audio_data):
        """Calculate energy in different frequency bands using FFT."""
        try:
            # Apply window to reduce spectral leakage
            window = np.hanning(len(audio_data))
            windowed = audio_data * window
            
            # Compute FFT
            fft = np.fft.rfft(windowed)
            fft_mag = np.abs(fft)
            
            # Divide into frequency bands
            freqs = np.fft.rfftfreq(len(audio_data), 1/SAMPLE_RATE)
            
            # Define frequency bands (Hz)
            bands = [
                (0, 250),      # Low bass
                (250, 500),    # Bass
                (500, 1000),   # Low mid
                (1000, 2000),  # Mid
                (2000, 3000),  # High mid
                (3000, 4000),  # Presence
                (4000, 6000),  # Brilliance
                (6000, 8000),  # High
            ]
            
            levels = []
            for low, high in bands:
                mask = (freqs >= low) & (freqs < high)
                if mask.any():
                    band_energy = np.mean(fft_mag[mask])
                    # Normalize to 0-1 range with log scale
                    level = np.log10(1 + band_energy * 10) / 2
                    levels.append(min(1.0, level))
                else:
                    levels.append(0)
            
            return np.array(levels)
        except Exception as e:
            return np.zeros(self.freq_bins)
    
    def create_level_bar(self, value, width=30, colored=True):
        """Create a visual level bar."""
        filled = int(value * width)
        bar = '█' * filled + '░' * (width - filled)
        
        if not colored:
            return bar
            
        # Color based on level
        if value < 0.2:
            color = BLUE
        elif value < 0.4:
            color = CYAN
        elif value < 0.6:
            color = GREEN
        elif value < 0.8:
            color = YELLOW
        else:
            color = RED
            
        return f"{color}{bar}{RESET}"
    
    def create_vad_indicator(self):
        """Create VAD status indicator with history."""
        if self.vad_active:
            indicator = f"{GREEN}● VOICE DETECTED{RESET}"
        else:
            indicator = f"{DIM}○ No Voice{RESET}"
            
        # Create history visualization
        history_str = ""
        for vad in self.vad_history:
            if vad:
                history_str += f"{GREEN}▪{RESET}"
            else:
                history_str += f"{DIM}·{RESET}"
                
        return indicator, history_str
    
    def display_visualization(self, audio_data):
        """Display real-time audio visualization."""
        current_time = time.time()
        if current_time - self.last_display_time < self.display_interval:
            return
        self.last_display_time = current_time
        
        # Calculate frequency bands
        self.freq_levels = self.calculate_frequency_bands(audio_data)
        
        # Clear previous lines (we'll use 10 lines for frequency spectrum)
        for _ in range(10):
            print(f"{CURSOR_UP}{CLEAR_LINE}", end="")
        
        # Frequency Spectrum
        print(f"{BOLD}Frequency Spectrum:{RESET}")
        print(f"{DIM}{'─'*50}{RESET}")
        freq_labels = ['<250', '250-500', '500-1k', '1k-2k', '2k-3k', '3k-4k', '4k-6k', '6k-8k']
        
        for i, (label, level) in enumerate(zip(freq_labels, self.freq_levels)):
            bar = self.create_level_bar(level, width=30)
            print(f"  {label:>7} Hz: {bar} {level:.2f}")
        
    def handle_audio(self, audio_array):
        """Process incoming audio data."""
        audio_data = audio_array.to_numpy()
        self.chunks_received += 1
        
        # Display visualization
        self.display_visualization(audio_data)
    
    def handle_vad(self, vad_array):
        """Process VAD status updates."""
        if len(vad_array) > 0:
            self.vad_active = vad_array[0].as_py()
            self.vad_history.append(self.vad_active)
    
    def run(self):
        """Run the sound visualizer."""
        print(f"\n{GREEN}Waiting for AEC audio output...{RESET}")
        print(f"Press Ctrl+C to stop\n")
        
        # Reserve space for visualization (10 lines for frequency spectrum)
        for _ in range(10):
            print()
        
        try:
            for event in self.node:
                if event["type"] == "INPUT":
                    if event["id"] == "audio":
                        self.handle_audio(event["value"])
                    elif event["id"] == "vad_status":
                        self.handle_vad(event["value"])
                        
                elif event["type"] == "STOP":
                    break
                    
        except KeyboardInterrupt:
            # Clear visualization lines
            for _ in range(10):
                print(f"{CURSOR_UP}{CLEAR_LINE}", end="")
                
            print(f"\n{GREEN}✓ Visualization stopped{RESET}")
            
        except Exception as e:
            print(f"\n{RED}Error: {e}{RESET}")

if __name__ == "__main__":
    visualizer = SoundVisualizer()
    visualizer.run()