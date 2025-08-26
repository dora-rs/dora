#!/usr/bin/env python3
"""
Play test audio file through speakers for AEC testing
"""

import os
import time
import wave
import numpy as np
import pyaudio
import pyarrow as pa
from dora import Node
from pathlib import Path


def main():
    node = Node()
    
    # Configuration
    audio_file = os.getenv("AUDIO_FILE", "./test_audio/harvard.wav")
    loop = os.getenv("LOOP", "false").lower() == "true"
    delay_start = int(os.getenv("DELAY_START", "5"))
    
    # Check if file exists - try multiple locations
    audio_path = Path(audio_file)
    if not audio_path.exists():
        # Try relative to voice-chatbot-aec
        alt_path = Path("../voice-chatbot-aec") / "harvard.wav"
        if alt_path.exists():
            audio_path = alt_path
        else:
            print(f"ERROR: Audio file not found: {audio_file}")
            print("Please ensure harvard.wav is in test_audio/ directory")
            return
    
    print(f"Will play {audio_path} in {delay_start} seconds...")
    time.sleep(delay_start)
    
    # Load audio
    try:
        with wave.open(str(audio_path), 'rb') as wf:
            sample_rate = wf.getframerate()
            channels = wf.getnchannels()
            frames = wf.readframes(wf.getnframes())
            
            # Convert to mono if stereo
            audio_data = np.frombuffer(frames, dtype=np.int16)
            if channels > 1:
                audio_data = audio_data.reshape(-1, channels)
                audio_data = np.mean(audio_data, axis=1).astype(np.int16)
            
            # Convert to float32
            audio_float = audio_data.astype(np.float32) / 32768.0
            
        print(f"Loaded {len(audio_float)/sample_rate:.2f} seconds of audio")
    except Exception as e:
        print(f"Failed to load audio: {e}")
        return
    
    # Initialize PyAudio
    p = pyaudio.PyAudio()
    
    chunk_size = 1024
    stream = p.open(
        format=pyaudio.paInt16,
        channels=1,
        rate=sample_rate,
        output=True,
        frames_per_buffer=chunk_size
    )
    
    try:
        play_count = 0
        while True:
            play_count += 1
            print(f"Playing audio (iteration {play_count})...")
            
            # Send playing status
            node.send_output("is_playing", pa.array([True]))
            
            # Play audio in chunks
            audio_int16 = (audio_float * 32768).astype(np.int16)
            
            for i in range(0, len(audio_int16), chunk_size):
                chunk = audio_int16[i:i+chunk_size]
                
                # Send what we're playing as reference
                chunk_float = chunk.astype(np.float32) / 32768.0
                node.send_output("audio", pa.array(chunk_float))
                
                # Play through speakers
                stream.write(chunk.tobytes())
                
                # Check for stop
                event = node.next(timeout=0)
                if event and event["type"] == "INPUT":
                    if event["id"] == "control" and event["value"][0].as_py() == "stop":
                        print("Received stop command")
                        stream.stop_stream()
                        stream.close()
                        p.terminate()
                        return
            
            # Send not playing status
            node.send_output("is_playing", pa.array([False]))
            print("Playback complete")
            
            if not loop:
                break
            
            time.sleep(2)  # Pause between loops
    
    finally:
        stream.stop_stream()
        stream.close()
        p.terminate()


if __name__ == "__main__":
    main()