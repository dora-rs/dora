#!/usr/bin/env python3
"""
Audio recorder node that captures audio from AEC and saves to WAV file.
Records for 20 seconds then saves the audio.
"""

import numpy as np
import wave
import time
from datetime import datetime
from dora import Node
import pyarrow as pa

class AudioRecorder:
    def __init__(self, duration_seconds=20, sample_rate=16000):
        self.duration_seconds = duration_seconds
        self.sample_rate = sample_rate
        self.audio_buffer = []
        self.start_time = None
        self.recording_complete = False
        self.vad_states = []
        
    def add_audio(self, audio_data):
        """Add audio data to buffer."""
        if self.start_time is None:
            self.start_time = time.time()
            print(f"Recording started at {datetime.now().strftime('%H:%M:%S')}")
            print(f"Recording for {self.duration_seconds} seconds...")
        
        self.audio_buffer.append(audio_data)
        
        elapsed = time.time() - self.start_time
        if elapsed >= self.duration_seconds and not self.recording_complete:
            self.save_recording()
            self.recording_complete = True
            return True
        
        # Print progress every 2 seconds
        if int(elapsed) % 2 == 0 and elapsed > 0:
            remaining = self.duration_seconds - elapsed
            if remaining > 0:
                print(f"Recording... {int(remaining)} seconds remaining")
        
        return False
    
    def add_vad_status(self, is_voice_active):
        """Track VAD status for debugging."""
        self.vad_states.append(is_voice_active)
    
    def save_recording(self):
        """Save the recorded audio to a WAV file."""
        if not self.audio_buffer:
            print("No audio data to save")
            return None
            
        # Concatenate all audio chunks
        audio_data = np.concatenate(self.audio_buffer)
        
        # Generate filename with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"aec_recording_{timestamp}.wav"
        
        # Convert to int16 for WAV file
        if audio_data.dtype == np.float32:
            audio_int16 = (audio_data * 32767).astype(np.int16)
        else:
            audio_int16 = audio_data.astype(np.int16)
        
        # Save as WAV file
        with wave.open(filename, 'wb') as wav_file:
            wav_file.setnchannels(1)  # Mono
            wav_file.setsampwidth(2)  # 16-bit
            wav_file.setframerate(self.sample_rate)
            wav_file.writeframes(audio_int16.tobytes())
        
        print(f"\n✓ Recording saved to: {filename}")
        print(f"  Duration: {len(audio_data) / self.sample_rate:.2f} seconds")
        print(f"  Samples: {len(audio_data)}")
        print(f"  Sample rate: {self.sample_rate} Hz")
        
        if self.vad_states:
            voice_active_count = sum(self.vad_states)
            print(f"  VAD: Voice detected in {voice_active_count}/{len(self.vad_states)} updates")
        
        return filename

def main():
    node = Node("audio-recorder")  # Add node ID for dynamic node
    recorder = AudioRecorder(duration_seconds=20, sample_rate=16000)
    
    print("AEC Microphone Test - Audio Recorder")
    print("=" * 40)
    print("Waiting for audio from AEC processor...")
    
    for event in node:
        if event["type"] == "INPUT":
            if event["id"] == "audio":
                # Extract audio data from Arrow array
                audio_array = event["value"].to_numpy()
                
                # Add to recorder
                recording_complete = recorder.add_audio(audio_array)
                
                if recording_complete:
                    # Send status update
                    status_msg = f"Recording complete. Saved to file."
                    node.send_output("status", pa.array([status_msg]))
                    node.send_output("log", pa.array([f"[Recorder] {status_msg}"]))
                    print("\nTest complete. Exiting...")
                    break
                    
            elif event["id"] == "vad_status":
                # Track VAD status
                vad_value = event["value"][0].as_py() if len(event["value"]) > 0 else False
                recorder.add_vad_status(vad_value)
                
        elif event["type"] == "STOP":
            print("\nStopping recorder...")
            if not recorder.recording_complete:
                recorder.save_recording()
            break

if __name__ == "__main__":
    main()