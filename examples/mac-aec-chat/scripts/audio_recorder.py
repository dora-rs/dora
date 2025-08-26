#!/usr/bin/env python3
"""
Audio recorder for offline analysis
Saves reference audio, AEC output, and VAD states to files
"""

import os
import time
import wave
import numpy as np
import pyarrow as pa
from dora import Node
import json
from pathlib import Path


class AudioRecorder:
    def __init__(self):
        self.node = Node()
        
        # Configuration
        self.output_dir = os.getenv("OUTPUT_DIR", "./echo_test_results")
        self.record_duration = int(os.getenv("RECORD_DURATION", "60"))
        self.sample_rate = 16000
        
        # Create output directory
        Path(self.output_dir).mkdir(parents=True, exist_ok=True)
        
        # Recording state
        self.start_time = time.time()
        self.recording = True
        
        # Audio buffers
        self.reference_audio = []
        self.aec_audio = []
        self.vad_states = []
        
        # File names with timestamp
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        self.reference_file = os.path.join(self.output_dir, f"reference_{timestamp}.wav")
        self.aec_file = os.path.join(self.output_dir, f"aec_output_{timestamp}.wav")
        self.vad_file = os.path.join(self.output_dir, f"vad_states_{timestamp}.json")
        self.log_file = os.path.join(self.output_dir, f"recording_log_{timestamp}.txt")
        
        print(f"Recording session started:")
        print(f"  Output directory: {self.output_dir}")
        print(f"  Duration: {self.record_duration}s")
        print(f"  Reference file: {self.reference_file}")
        print(f"  AEC file: {self.aec_file}")
        print(f"  VAD file: {self.vad_file}")
    
    def save_audio_file(self, filename, audio_data):
        """Save audio data to WAV file"""
        if not audio_data:
            print(f"Warning: No audio data to save for {filename}")
            return
        
        try:
            # Convert to int16
            audio_array = np.array(audio_data, dtype=np.float32)
            audio_int16 = (audio_array * 32767).astype(np.int16)
            
            with wave.open(filename, 'wb') as wf:
                wf.setnchannels(1)  # Mono
                wf.setsampwidth(2)  # 16-bit
                wf.setframerate(self.sample_rate)
                wf.writeframes(audio_int16.tobytes())
                
            duration = len(audio_int16) / self.sample_rate
            print(f"Saved {filename}: {duration:.2f}s, {len(audio_int16)} samples")
            
        except Exception as e:
            print(f"Error saving {filename}: {e}")
    
    def save_vad_data(self):
        """Save VAD states to JSON file"""
        try:
            vad_data = {
                'recording_info': {
                    'start_time': self.start_time,
                    'duration': time.time() - self.start_time,
                    'sample_rate': self.sample_rate,
                    'total_vad_samples': len(self.vad_states)
                },
                'vad_states': self.vad_states
            }
            
            with open(self.vad_file, 'w') as f:
                json.dump(vad_data, f, indent=2)
                
            print(f"Saved VAD data: {len(self.vad_states)} states")
            
        except Exception as e:
            print(f"Error saving VAD data: {e}")
    
    def save_recording_log(self):
        """Save recording session log"""
        try:
            duration = time.time() - self.start_time
            ref_duration = len(self.reference_audio) / self.sample_rate if self.reference_audio else 0
            aec_duration = len(self.aec_audio) / self.sample_rate if self.aec_audio else 0
            
            log_content = f"""Echo Cancellation Test Recording Log
=====================================

Recording Session:
  Start time: {time.ctime(self.start_time)}
  Duration: {duration:.2f} seconds
  Sample rate: {self.sample_rate} Hz

Audio Files:
  Reference: {self.reference_file} ({ref_duration:.2f}s, {len(self.reference_audio)} samples)
  AEC Output: {self.aec_file} ({aec_duration:.2f}s, {len(self.aec_audio)} samples)
  VAD States: {self.vad_file} ({len(self.vad_states)} states)

Analysis:
  Reference RMS: {np.sqrt(np.mean(np.array(self.reference_audio)**2)):.4f if self.reference_audio else 0:.4f}
  AEC Output RMS: {np.sqrt(np.mean(np.array(self.aec_audio)**2)):.4f if self.aec_audio else 0:.4f}
  VAD Active Ratio: {sum(1 for _, active in self.vad_states if active) / len(self.vad_states) * 100 if self.vad_states else 0:.1f}%

Files can be analyzed with tools like:
  - ffplay {os.path.basename(self.reference_file)}
  - ffplay {os.path.basename(self.aec_file)}
  - python analyze_recordings.py {os.path.basename(self.reference_file)} {os.path.basename(self.aec_file)}
"""
            
            with open(self.log_file, 'w') as f:
                f.write(log_content)
                
            print(f"Saved recording log: {self.log_file}")
            
        except Exception as e:
            print(f"Error saving log: {e}")
    
    def finalize_recording(self):
        """Save all recorded data to files"""
        print("\nFinalizing recording...")
        
        # Save audio files
        self.save_audio_file(self.reference_file, self.reference_audio)
        self.save_audio_file(self.aec_file, self.aec_audio)
        
        # Save VAD data
        self.save_vad_data()
        
        # Save log
        self.save_recording_log()
        
        print(f"\nRecording complete! Files saved to: {self.output_dir}")
    
    def run(self):
        """Main recording loop"""
        print(f"Starting recording for {self.record_duration} seconds...")
        print("Press Ctrl+C to stop early")
        
        try:
            while self.recording:
                event = self.node.next(timeout=0.1)
                
                if event and event["type"] == "INPUT":
                    current_time = time.time()
                    
                    if event["id"] == "reference":
                        # Record reference audio (what's playing through speakers)
                        audio_data = event["value"].to_numpy()
                        self.reference_audio.extend(audio_data)
                        
                    elif event["id"] == "aec_output":
                        # Record AEC processed audio (what we capture after echo cancellation)
                        audio_data = event["value"].to_numpy()
                        self.aec_audio.extend(audio_data)
                        
                    elif event["id"] == "vad":
                        # Record VAD state changes
                        vad_active = event["value"][0].as_py()
                        self.vad_states.append((current_time - self.start_time, vad_active))
                
                # Check if recording time is up
                if time.time() - self.start_time >= self.record_duration:
                    print(f"\nRecording time limit reached ({self.record_duration}s)")
                    self.recording = False
                
                # Send status updates
                elapsed = time.time() - self.start_time
                remaining = max(0, self.record_duration - elapsed)
                status = {
                    'recording': self.recording,
                    'elapsed': elapsed,
                    'remaining': remaining,
                    'progress': elapsed / self.record_duration * 100
                }
                self.node.send_output("recording_status", pa.array([json.dumps(status)]))
                
        except KeyboardInterrupt:
            print(f"\nRecording interrupted by user")
            self.recording = False
            
        except Exception as e:
            print(f"Recording error: {e}")
            self.recording = False
            
        finally:
            self.finalize_recording()


if __name__ == "__main__":
    recorder = AudioRecorder()
    recorder.run()