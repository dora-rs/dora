#!/usr/bin/env python3
"""
Circular Buffer Audio Player with Backpressure Control.
Sends buffer fullness percentage for external flow control.
"""

import os
import time
import threading
import signal
import sys
import numpy as np
import pyarrow as pa
from dora import Node
import sounddevice as sd


class CircularAudioBuffer:
    """Thread-safe circular buffer for audio streaming."""

    def __init__(self, size_seconds=60, sample_rate=32000):
        self.sample_rate = sample_rate
        self.buffer_size = int(size_seconds * sample_rate)
        self.buffer = np.zeros(self.buffer_size, dtype=np.float32)
        
        self.write_pos = 0
        self.read_pos = 0
        self.available_samples = 0
        
        self.lock = threading.Lock()
        
        self.total_written = 0
        self.total_read = 0
        self.underruns = 0
        self.overruns = 0

    def write(self, audio_data: np.ndarray) -> int:
        with self.lock:
            data_len = len(audio_data)
            
            if self.available_samples + data_len > self.buffer_size:
                self.overruns += 1
                overflow = (self.available_samples + data_len) - self.buffer_size
                self.read_pos = (self.read_pos + overflow) % self.buffer_size
                self.available_samples -= overflow
                print(f"[Buffer] Overrun! Skipping {overflow} samples")
            
            samples_written = 0
            while samples_written < data_len:
                chunk_size = min(data_len - samples_written, self.buffer_size - self.write_pos)
                end_pos = self.write_pos + chunk_size
                self.buffer[self.write_pos:end_pos] = audio_data[samples_written:samples_written + chunk_size]
                self.write_pos = end_pos % self.buffer_size
                samples_written += chunk_size
            
            self.available_samples = min(self.available_samples + data_len, self.buffer_size)
            self.total_written += data_len
            return data_len

    def read(self, num_samples: int) -> np.ndarray:
        with self.lock:
            if self.available_samples < num_samples:
                # Partial read - return what we have plus zeros
                actual_samples = self.available_samples
                output = np.zeros(num_samples, dtype=np.float32)
                
                if actual_samples > 0:
                    # Read all available samples
                    samples_read = 0
                    while samples_read < actual_samples:
                        chunk_size = min(actual_samples - samples_read, self.buffer_size - self.read_pos)
                        end_pos = self.read_pos + chunk_size
                        output[samples_read:samples_read + chunk_size] = self.buffer[self.read_pos:end_pos]
                        self.read_pos = end_pos % self.buffer_size
                        samples_read += chunk_size
                    
                    self.available_samples = 0  # Buffer is now empty
                    self.total_read += actual_samples
                
                self.underruns += 1
                return output
            
            output = np.zeros(num_samples, dtype=np.float32)
            samples_read = 0
            
            while samples_read < num_samples:
                chunk_size = min(num_samples - samples_read, self.buffer_size - self.read_pos)
                end_pos = self.read_pos + chunk_size
                output[samples_read:samples_read + chunk_size] = self.buffer[self.read_pos:end_pos]
                self.read_pos = end_pos % self.buffer_size
                samples_read += chunk_size
            
            self.available_samples -= num_samples
            self.total_read += num_samples
            return output

    def reset(self):
        """Reset the buffer to empty state."""
        with self.lock:
            self.buffer.fill(0)  # Clear buffer data
            self.write_pos = 0
            self.read_pos = 0
            self.available_samples = 0
            self.total_written = 0
            self.total_read = 0
            self.underruns = 0
            self.overruns = 0
    
    def get_stats(self):
        with self.lock:
            return {
                'available_samples': self.available_samples,
                'available_seconds': self.available_samples / self.sample_rate,
                'buffer_fill': (self.available_samples / self.buffer_size) * 100,
                'total_written': self.total_written,
                'total_read': self.total_read,
                'underruns': self.underruns,
                'overruns': self.overruns
            }


class CircularBufferAudioPlayer:
    """Audio player using circular buffer."""

    def __init__(self, buffer_seconds=60, sample_rate=32000, blocksize=2048):
        self.sample_rate = sample_rate
        self.blocksize = blocksize
        self.buffer = CircularAudioBuffer(buffer_seconds, sample_rate)
        self.playing = False
        self.paused = True
        self.stream = None

    def audio_callback(self, outdata, frames, time_info, status):
        if status:
            print(f"[Audio] Callback status: {status}")
        if self.paused:
            outdata[:] = 0
        else:
            audio_data = self.buffer.read(frames)
            outdata[:] = audio_data.reshape(-1, 1)

    def start(self):
        if not self.playing:
            self.stream = sd.OutputStream(
                samplerate=self.sample_rate,
                channels=1,
                dtype='float32',
                callback=self.audio_callback,
                blocksize=self.blocksize
            )
            self.stream.start()
            self.playing = True
            print(f"[Audio Player] Started at {self.sample_rate}Hz")

    def stop(self):
        if self.playing and self.stream:
            self.stream.stop()
            self.stream.close()
            self.playing = False

    def pause(self):
        self.paused = True

    def resume(self):
        self.paused = False

    def add_audio(self, audio_data):
        if audio_data is None or len(audio_data) == 0:
            return 0
        if not isinstance(audio_data, np.ndarray):
            audio_data = np.array(audio_data, dtype=np.float32)
        return self.buffer.write(audio_data)
    
    def reset(self):
        """Reset the audio buffer and pause playback."""
        self.pause()
        self.buffer.reset()
        print("[Audio Player] Buffer reset to empty")


shutdown_flag = threading.Event()

def signal_handler(signum, frame):
    print("\n[Audio Player] Shutting down...")
    shutdown_flag.set()


def main():
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        node = Node("audio-player")
        player = CircularBufferAudioPlayer(buffer_seconds=60, sample_rate=32000, blocksize=2048)
        player.start()

        # Clear screen
        print("\033[2J\033[H", end="")
        
        print("=" * 60)
        print("CIRCULAR BUFFER AUDIO PLAYER WITH CONTROL")
        print("=" * 60)
        print("Buffer: 60 seconds | Outputs buffer percentage")
        print("Sends status for backpressure control")
        print("=" * 60 + "\n")

        # State
        segments_received = 0
        start_time = time.time()
        playback_started = False
        discard_next_audio = False  # Flag to discard next audio after reset
        
        # Timing
        last_visualization_time = time.time()
        last_status_time = time.time()
        visualization_interval = 0.5
        
        # Get configurable timeout from environment (in milliseconds)
        node_timeout_ms = int(os.getenv("NODE_TIMEOUT_MS", "1000"))
        node_timeout = node_timeout_ms / 1000.0  # Convert to seconds

        while not shutdown_flag.is_set():
            # Process events with configurable timeout
            try:
                event = node.next(timeout=node_timeout)  # Configurable timeout for responsive buffer updates
            except KeyboardInterrupt:
                break
            except Exception:
                continue
            
            # Handle control input
            if event and event["type"] == "INPUT" and event["id"] == "control":
                try:
                    control_cmd = event["value"][0].as_py()
                    metadata = event.get("metadata", {})
                    
                    if control_cmd == "reset":
                        # Reset the buffer
                        player.reset()
                        player.pause()  # Make sure we're paused after reset
                        playback_started = False
                        segments_received = 0
                        
                        # Set flag to discard next audio chunk from TTS
                        discard_next_audio = True
                        
                        elapsed = time.time() - start_time
                        # Don't print inline messages that would mess up the display
                    elif control_cmd == "pause":
                        player.pause()
                        # Don't print inline messages that would mess up the display
                        pass
                    elif control_cmd == "resume":
                        player.resume()
                        # Don't print inline messages that would mess up the display
                        pass
                    elif control_cmd == "status":
                        stats = player.buffer.get_stats()
                        print(f"[{time.time() - start_time:6.2f}s] Buffer Status:")
                        print(f"  Available: {stats['available_seconds']:.1f}s ({stats['buffer_fill']:.1f}%)")
                        print(f"  Underruns: {stats['underruns']}, Overruns: {stats['overruns']}")
                except Exception as e:
                    print(f"[Error] Processing control signal: {e}")
            
            # Handle audio input
            elif event and event["type"] == "INPUT" and event["id"] == "audio":
                try:
                    raw_value = event.get("value")
                    if raw_value and len(raw_value) > 0:
                        audio_data = raw_value[0].as_py()
                        if audio_data is not None:
                            if not isinstance(audio_data, np.ndarray):
                                audio_data = np.array(audio_data, dtype=np.float32)
                            
                            if len(audio_data) > 0:
                                # Check if we should discard this audio due to reset
                                if discard_next_audio:
                                    metadata = event.get("metadata", {})
                                    fragment_num = metadata.get("fragment_num", 0)
                                    segment_index = metadata.get("segment_index", -1)
                                    
                                    # Check if this is the start of a new synthesis (fragment 1 or segment 0)
                                    if fragment_num == 1 or segment_index == 0:
                                        # This is new synthesis, stop discarding
                                        discard_next_audio = False
                                        # Don't print inline messages that would mess up the display
                                        pass
                                    else:
                                        # Still old synthesis, discard
                                        # Don't print inline messages that would mess up the display
                                        continue  # Skip processing this audio
                                
                                metadata = event.get("metadata", {})
                                segment_index = metadata.get("segment_index", -1)
                                duration = len(audio_data) / 32000.0
                                
                                segments_received += 1
                                # Don't print inline messages that would mess up the display
                                
                                player.add_audio(audio_data)
                                
                                # Auto-start playback as soon as we have any audio
                                if not playback_started and len(audio_data) > 0:
                                    player.resume()  # Start playing immediately
                                    playback_started = True
                                    # Don't print inline messages that would mess up the display
                                    pass
                                
                                # No buffer management - just keep playing whatever is in the buffer
                                # The audio callback will handle underruns gracefully
                except Exception as e:
                    print(f"[Error] {e}")
            
            elif event and event["type"] == "STOP":
                break
            
            current_time = time.time()
            
            # Send buffer status for flow control
            stats = player.buffer.get_stats()
            buffer_percentage = stats['buffer_fill']
            
            # Send buffer status at the rate determined by node.next timeout
            # No need for complex interval logic since node.next controls the frequency
            if current_time - last_status_time >= node_timeout:
                # Send buffer percentage as status output
                try:
                    node.send_output(
                        "buffer_status",
                        pa.array([buffer_percentage]),
                        metadata={
                            "buffer_percentage": buffer_percentage,
                            "buffer_seconds": stats['available_seconds'],
                            "underruns": stats['underruns'],
                            "overruns": stats['overruns'],
                            "timestamp": current_time
                        }
                    )
                    
                    # Don't print inline messages that would mess up the display
                    pass
                except Exception:
                    pass
                
                last_status_time = current_time
            
            # Display visualization (less frequent)
            if current_time - last_visualization_time >= visualization_interval:
                stats = player.buffer.get_stats()
                elapsed = current_time - start_time
                
                # Buffer metrics
                buffer_seconds = stats['available_seconds']
                buffer_percent = stats['buffer_fill']
                
                # FIXED: Proper alignment - 40 chars total for 60s
                # 40 chars / 60s = 0.667 chars per second
                # So we need chars_filled = buffer_seconds * 0.667
                bar_width = 40
                chars_per_second = bar_width / 60.0
                chars_filled = int(buffer_seconds * chars_per_second)
                
                # Build properly aligned buffer bar
                buffer_bar = "█" * min(chars_filled, bar_width) + "░" * max(0, bar_width - chars_filled)
                
                # Status
                if buffer_percent < 5:
                    status = "EMPTY"
                    icon = "⚠️"
                elif buffer_percent < 20:
                    status = "LOW"
                    icon = "⚠️"
                elif buffer_percent > 80:
                    status = "HIGH"
                    icon = "⚠️"
                else:
                    status = "NORMAL"
                    icon = "✓"
                
                playback_status = "PLAYING" if (playback_started and not player.paused) else "PAUSED" if player.paused else "WAITING"
                
                # Move cursor to line 8 (after headers) and clear from there
                print("\033[8;1H\033[J", end="")
                
                # Display without box - single updating display
                print(f"[{elapsed:7.1f}s] BUFFER: {status} {icon}")
                print(f"─" * 60)
                print(f"Buffer: [{buffer_bar}] {buffer_percent:5.1f}%")
                print(f"        └{'─' * 9}┴{'─' * 9}┴{'─' * 9}┴{'─' * 9}┘")
                print(f"        0s       15s      30s      45s      60s")
                print(f"─" * 60)
                print(f"Buffer: {buffer_seconds:5.1f}s / 60s | Status: {playback_status}")
                print(f"Segments: {segments_received:4d} | Control: {buffer_percent:5.1f}% full")
                
                last_visualization_time = current_time

    except Exception as e:
        print(f"[Fatal] {e}")
    finally:
        if 'player' in locals():
            player.stop()
        print("[Audio Player] Stopped")


if __name__ == "__main__":
    main()