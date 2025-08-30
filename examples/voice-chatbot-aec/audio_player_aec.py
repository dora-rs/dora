#!/usr/bin/env python3
"""
Simplified Audio Player for AEC Voice Chatbot

This version doesn't need buffer status reporting since AEC handles
echo cancellation automatically.
"""

import time
import threading
from collections import deque
from typing import Optional

import numpy as np
import pyarrow as pa
import sounddevice as sd
from dora import Node


class CircularAudioBuffer:
    """Thread-safe circular buffer for audio samples"""
    
    def __init__(self, capacity_seconds: float = 10.0, sample_rate: int = 22050):
        self.capacity = int(capacity_seconds * sample_rate)
        self.buffer = deque(maxlen=self.capacity)
        self.sample_rate = sample_rate
        self.lock = threading.Lock()
    
    def write(self, samples: np.ndarray):
        """Add samples to the buffer"""
        with self.lock:
            self.buffer.extend(samples)
    
    def read(self, num_samples: int) -> Optional[np.ndarray]:
        """Read and remove samples from the buffer"""
        with self.lock:
            if len(self.buffer) >= num_samples:
                samples = [self.buffer.popleft() for _ in range(num_samples)]
                return np.array(samples, dtype=np.float32)
            elif len(self.buffer) > 0:
                # Return all remaining samples
                samples = list(self.buffer)
                self.buffer.clear()
                return np.array(samples, dtype=np.float32)
            return None
    
    def clear(self):
        """Clear the buffer"""
        with self.lock:
            self.buffer.clear()
    
    def __len__(self):
        """Get current buffer size"""
        with self.lock:
            return len(self.buffer)


class AudioPlayer:
    """Simplified audio player without buffer status reporting"""
    
    def __init__(self, node: Node):
        self.node = node
        
        # Load configuration from environment variables (not node.metadata)
        import os
        self.sample_rate = int(os.getenv("SAMPLE_RATE", "22050"))
        self.channels = int(os.getenv("CHANNELS", "1"))
        self.chunk_size = int(os.getenv("CHUNK_SIZE", "1024"))
        
        # Audio buffer
        self.buffer = CircularAudioBuffer(
            capacity_seconds=10.0,
            sample_rate=self.sample_rate
        )
        
        # Playback state
        self.is_playing = False
        self.stream: Optional[sd.OutputStream] = None
        self.playback_thread: Optional[threading.Thread] = None
        
        self._log_info(f"Audio player initialized: {self.sample_rate}Hz, {self.channels} channel(s)")
    
    def start_playback(self):
        """Start audio playback"""
        if self.is_playing:
            return
        
        self.is_playing = True
        
        # Create audio stream
        self.stream = sd.OutputStream(
            samplerate=self.sample_rate,
            channels=self.channels,
            dtype=np.float32,
            blocksize=self.chunk_size,
            callback=self._audio_callback
        )
        
        self.stream.start()
        self._log_info("Audio playback started")
    
    def stop_playback(self):
        """Stop audio playback"""
        if not self.is_playing:
            return
        
        self.is_playing = False
        
        if self.stream:
            self.stream.stop()
            self.stream.close()
            self.stream = None
        
        self._log_info("Audio playback stopped")
    
    def _audio_callback(self, outdata, frames, time_info, status):
        """Audio stream callback"""
        if status:
            self._log_warning(f"Audio stream status: {status}")
        
        # Read samples from buffer
        samples = self.buffer.read(frames)
        
        if samples is not None:
            # Pad with zeros if needed
            if len(samples) < frames:
                samples = np.pad(samples, (0, frames - len(samples)), mode='constant')
            
            # Write to output
            if self.channels == 1:
                outdata[:, 0] = samples
            else:
                outdata[:] = samples.reshape(-1, self.channels)
        else:
            # No audio available, output silence
            outdata.fill(0)
    
    def handle_audio(self, audio_data: np.ndarray):
        """Handle incoming audio data"""
        # Convert to float32 if needed
        if audio_data.dtype != np.float32:
            audio_data = audio_data.astype(np.float32)
        
        # Add to buffer
        self.buffer.write(audio_data)
        
        # Start playback if not already playing
        if not self.is_playing:
            self.start_playback()
    
    def _log_info(self, message: str):
        """Send info log message"""
        self.node.send_output("log", pa.array([f"[INFO ] Audio Player: {message}"]))
    
    def _log_warning(self, message: str):
        """Send warning log message"""
        self.node.send_output("log", pa.array([f"[WARN ] Audio Player: {message}"]))
    
    def _log_error(self, message: str):
        """Send error log message"""
        self.node.send_output("log", pa.array([f"[ERROR] Audio Player: {message}"]))


def main():
    """Main entry point"""
    node = Node("audio-player")
    player = AudioPlayer(node)
    
    player._log_info("Audio player ready (AEC version)")
    
    try:
        while True:
            event = node.next(timeout=10.0)
            
            if event is None:
                continue
            
            if event["type"] == "INPUT":
                input_id = event.get("id", "")
                
                if input_id == "audio":
                    # Handle audio input
                    audio_data = event["value"].to_numpy()
                    player.handle_audio(audio_data)
                    
    except KeyboardInterrupt:
        player._log_info("Shutting down...")
    except Exception as e:
        player._log_error(f"Unexpected error: {e}")
    finally:
        player.stop_playback()


if __name__ == "__main__":
    main()