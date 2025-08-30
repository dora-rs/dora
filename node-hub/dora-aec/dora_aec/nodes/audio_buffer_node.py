#!/usr/bin/env python3
"""
Audio Buffer Node - Accumulates audio chunks with VAD awareness
"""

import numpy as np
import pyarrow as pa
from dora import Node


class AudioBuffer:
    def __init__(self, sample_rate=16000, buffer_duration=1.0):
        self.sample_rate = sample_rate
        self.buffer_size = int(sample_rate * buffer_duration)
        self.buffer = []
        self.total_samples = 0
        self.voice_active = False
    
    def add(self, audio_chunk, vad_status=True):
        """Add audio to buffer"""
        self.buffer.append(audio_chunk)
        self.total_samples += len(audio_chunk)
        self.voice_active = self.voice_active or vad_status
        
        # Return buffer if ready
        if self.total_samples >= self.buffer_size:
            return self.flush()
        return None
    
    def flush(self):
        """Get buffered audio and reset"""
        if not self.buffer:
            return None
        
        # Concatenate all chunks
        audio = np.concatenate(self.buffer)
        voice_was_active = self.voice_active
        
        # Reset buffer
        self.buffer = []
        self.total_samples = 0
        self.voice_active = False
        
        return audio, voice_was_active


def main():
    node = Node()
    buffer = AudioBuffer()
    current_vad = False
    
    while True:
        event = node.next()
        
        if event["type"] == "INPUT":
            if event["id"] == "audio":
                audio_chunk = event["value"].to_numpy()
                
                # Add to buffer with current VAD status
                result = buffer.add(audio_chunk, current_vad)
                
                if result:
                    buffered_audio, had_voice = result
                    node.send_output("buffered_audio", pa.array(buffered_audio))
                    node.send_output("buffer_ready", pa.array([True]))
                    
            elif event["id"] == "vad":
                current_vad = event["value"][0].as_py()


if __name__ == "__main__":
    main()