#!/usr/bin/env python3
"""
Audio Segment Recorder Node
Records audio segments from MAC-AEC to WAV files for debugging
"""

import os
import sys
import time
import wave
import numpy as np
import pyarrow as pa
from dora import Node
from datetime import datetime
from pathlib import Path
from collections import deque


class AudioSegmentRecorder:
    """Records audio segments to WAV files for analysis"""
    
    def __init__(self, output_dir="./aec_recordings", duration_seconds=30):
        """
        Initialize the recorder
        
        Args:
            output_dir: Directory to save recordings
            duration_seconds: How long to record (default 30s)
        """
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        self.duration_seconds = duration_seconds
        self.sample_rate = 16000
        
        # Buffers for different audio streams
        self.audio_segments = []  # Complete segments from AEC
        self.raw_audio_buffer = deque(maxlen=self.sample_rate * duration_seconds)  # Rolling buffer for raw audio
        
        # Recording state
        self.start_time = time.time()
        self.segment_count = 0
        self.raw_frame_count = 0
        
        # Session ID for this recording
        self.session_id = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        print(f"[RECORDER] Initialized - Session: {self.session_id}")
        print(f"[RECORDER] Recording for {duration_seconds} seconds")
        print(f"[RECORDER] Output directory: {self.output_dir}")
    
    def save_audio_segment(self, audio_data, segment_num):
        """Save a single audio segment to WAV"""
        filename = self.output_dir / f"segment_{self.session_id}_{segment_num:04d}.wav"
        
        # Convert float32 to int16 for WAV
        if audio_data.dtype == np.float32:
            audio_int16 = (audio_data * 32768).clip(-32768, 32767).astype(np.int16)
        else:
            audio_int16 = audio_data
        
        # Write WAV file
        with wave.open(str(filename), 'wb') as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)  # 16-bit
            wf.setframerate(self.sample_rate)
            wf.writeframes(audio_int16.tobytes())
        
        duration = len(audio_data) / self.sample_rate
        print(f"[RECORDER] Saved segment {segment_num}: {filename.name} ({duration:.2f}s)")
        
        return filename
    
    def save_continuous_audio(self):
        """Save the continuous raw audio buffer"""
        if len(self.raw_audio_buffer) == 0:
            print("[RECORDER] No raw audio to save")
            return
        
        filename = self.output_dir / f"continuous_{self.session_id}.wav"
        
        # Convert deque to numpy array
        audio_array = np.array(self.raw_audio_buffer, dtype=np.float32)
        
        # Convert to int16
        audio_int16 = (audio_array * 32768).clip(-32768, 32767).astype(np.int16)
        
        # Write WAV file
        with wave.open(str(filename), 'wb') as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(self.sample_rate)
            wf.writeframes(audio_int16.tobytes())
        
        duration = len(audio_array) / self.sample_rate
        print(f"[RECORDER] Saved continuous audio: {filename.name} ({duration:.2f}s)")
        
        return filename
    
    def save_combined_segments(self):
        """Combine all segments into one file with silence between them"""
        if not self.audio_segments:
            print("[RECORDER] No segments to combine")
            return
        
        filename = self.output_dir / f"combined_segments_{self.session_id}.wav"
        
        # Add 0.5s silence between segments
        silence = np.zeros(int(self.sample_rate * 0.5), dtype=np.float32)
        
        combined = []
        for i, segment in enumerate(self.audio_segments):
            combined.extend(segment)
            if i < len(self.audio_segments) - 1:
                combined.extend(silence)
        
        audio_array = np.array(combined, dtype=np.float32)
        audio_int16 = (audio_array * 32768).clip(-32768, 32767).astype(np.int16)
        
        with wave.open(str(filename), 'wb') as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(self.sample_rate)
            wf.writeframes(audio_int16.tobytes())
        
        duration = len(audio_array) / self.sample_rate
        print(f"[RECORDER] Saved combined segments: {filename.name} ({duration:.2f}s, {len(self.audio_segments)} segments)")
        
        return filename
    
    def process_audio_segment(self, audio_data):
        """Process an audio segment from AEC"""
        self.segment_count += 1
        
        # Save individual segment
        self.save_audio_segment(audio_data, self.segment_count)
        
        # Store for combined file
        self.audio_segments.append(audio_data)
        
        # Add to continuous buffer
        self.raw_audio_buffer.extend(audio_data)
    
    def process_raw_audio(self, audio_data):
        """Process raw audio frames for continuous recording"""
        self.raw_frame_count += 1
        
        # Add to rolling buffer
        self.raw_audio_buffer.extend(audio_data)
        
        # Log progress every 30 frames (about 1 second)
        if self.raw_frame_count % 30 == 0:
            buffer_duration = len(self.raw_audio_buffer) / self.sample_rate
            max_val = np.abs(audio_data).max() if len(audio_data) > 0 else 0
            print(f"[RECORDER] Frame {self.raw_frame_count}: buffer={buffer_duration:.1f}s, max_amp={max_val:.4f}")
    
    def is_recording_complete(self):
        """Check if recording duration has elapsed"""
        return (time.time() - self.start_time) >= self.duration_seconds
    
    def finalize(self):
        """Save final files and print summary"""
        print("\n" + "=" * 60)
        print("[RECORDER] Recording complete - Saving files...")
        print("=" * 60)
        
        # Save continuous audio
        continuous_file = self.save_continuous_audio()
        
        # Save combined segments
        combined_file = self.save_combined_segments()
        
        # Print summary
        print("\n[RECORDER] Summary:")
        print(f"  - Session ID: {self.session_id}")
        print(f"  - Duration: {time.time() - self.start_time:.1f}s")
        print(f"  - Segments recorded: {self.segment_count}")
        print(f"  - Raw frames: {self.raw_frame_count}")
        print(f"  - Output directory: {self.output_dir}")
        
        if continuous_file:
            print(f"  - Continuous audio: {continuous_file.name}")
        if combined_file:
            print(f"  - Combined segments: {combined_file.name}")
        
        print("\n[RECORDER] Individual segment files:")
        for i in range(1, self.segment_count + 1):
            filename = f"segment_{self.session_id}_{i:04d}.wav"
            print(f"  - {filename}")
        
        print("\n[RECORDER] Recording session complete!")
        print("=" * 60)


def main():
    """Main node loop"""
    node = Node("audio-recorder")
    
    # Get configuration from environment
    output_dir = os.getenv("OUTPUT_DIR", "./aec_recordings")
    duration_seconds = int(os.getenv("RECORD_DURATION", "30"))
    
    print("\n" + "=" * 60)
    print("Audio Segment Recorder Node")
    print("=" * 60)
    print(f"Recording AEC output for {duration_seconds} seconds")
    print(f"Output directory: {output_dir}")
    print("Waiting for audio segments...")
    print("=" * 60 + "\n")
    
    # Initialize recorder
    recorder = AudioSegmentRecorder(output_dir, duration_seconds)
    
    try:
        while True:
            # Check for timeout with 100ms
            event = node.next(timeout=0.1)
            
            if event and event["type"] == "INPUT":
                input_id = event["id"]
                
                if input_id == "audio_segment":
                    # Get audio segment from AEC
                    audio_array = event["value"].to_numpy()
                    
                    print(f"[RECORDER] Received segment: {len(audio_array)} samples ({len(audio_array)/16000:.2f}s)")
                    recorder.process_audio_segment(audio_array)
                    
                elif input_id == "audio":
                    # Get raw audio frames
                    audio_array = event["value"].to_numpy()
                    recorder.process_raw_audio(audio_array)
                    
                elif input_id == "control":
                    # Handle control commands
                    command = event["value"][0].as_py()
                    if command == "stop":
                        print("[RECORDER] Received stop command")
                        break
            
            # Check if recording is complete
            if recorder.is_recording_complete():
                print(f"\n[RECORDER] {duration_seconds} seconds elapsed - stopping recording")
                break
                
    except KeyboardInterrupt:
        print("\n[RECORDER] Interrupted by user")
    except Exception as e:
        print(f"[RECORDER] Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Save all recordings
        recorder.finalize()


if __name__ == "__main__":
    main()