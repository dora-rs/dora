#!/usr/bin/env python3
"""
MAC-AEC Simple Segmentation Wrapper
Wraps the dora-mac-aec node and adds VAD-based segmentation
"""

import os
import sys
import time
import numpy as np
import pyarrow as pa
from dora import Node
from typing import Optional
from collections import deque
from pathlib import Path
import json


def send_log(node, level, message):
    """Send log message through node output"""
    try:
        log_data = {
            "level": level,
            "message": message,
            "timestamp": time.time(),
            "node": "mac-aec"
        }
        node.send_output("log", pa.array([json.dumps(log_data)]))
    except:
        # Fallback to print if node not available
        print(f"[MAC-AEC] [{level}] {message}")


# Add node-hub to path for imports
node_hub_path = Path(__file__).parent / "../../node-hub"
if node_hub_path.exists():
    sys.path.insert(0, str(node_hub_path))

# Try to import from dora-aec (the working version with proper AEC)
dora_aec_path = Path(__file__).parent / "../../node-hub/dora-aec"
if dora_aec_path.exists():
    sys.path.insert(0, str(dora_aec_path))
    from dora_aec.aec_wrapper import AECWrapper
    # Silently use dora-aec
else:
    # Fallback to dora-mac-aec
    mac_aec_path = Path(__file__).parent / "../../node-hub/dora-mac-aec"
    if mac_aec_path.exists():
        sys.path.insert(0, str(mac_aec_path))
    from dora_mac_aec.aec_wrapper import AECWrapper
    from dora_mac_aec.utils.logger import AECLogger
    # Warning logged later when node is available


class MacAECSegmentation:
    """
    Wrapper for MAC-AEC that adds segmentation based on VAD
    """
    
    def __init__(self, node=None):
        """Initialize the MAC-AEC wrapper with segmentation"""
        self.node = node
        self.aec = None
        self.logger = None
        
        # VAD state tracking
        self.is_speaking = False
        self.speech_buffer = []
        self.silence_count = 0
        
        # Thresholds
        self.speech_start_threshold = 3  # Frames of speech to start
        self.speech_end_threshold = 10   # Frames of silence to end
        self.min_segment_size = 4800     # Minimum samples (0.3s at 16kHz)
        self.max_segment_size = 160000    # Maximum samples (10s at 16kHz)
        
        # Audio buffer for segmentation
        self.audio_segment_buffer = []
        self.sample_rate = 16000
        
        # Debug counter
        self._debug_counter = 0
        
        # Initialize MAC-AEC
        # Note: dora-aec doesn't use AECLogger, just print directly
        
        # Find the native library - check multiple locations
        # 1. First check local lib directory (for standalone distribution)
        lib_path = Path(__file__).parent / "lib/libAudioCapture.dylib"
        
        if not lib_path.exists():
            # 2. Try dora-mac-aec node location
            lib_path = Path(__file__).parent / "../../node-hub/dora-mac-aec/dora_mac_aec/lib/libAudioCapture.dylib"
        
        if not lib_path.exists():
            # 3. Try dora-aec node location
            lib_path = Path(__file__).parent / "../../node-hub/dora-aec/dora_aec/lib/libAudioCapture.dylib"
        
        if not lib_path.exists():
            raise FileNotFoundError(
                f"MAC-AEC native library not found. Please ensure libAudioCapture.dylib is in one of:\n"
                f"  1. {Path(__file__).parent / 'lib/'}\n"
                f"  2. {Path(__file__).parent / '../../node-hub/dora-mac-aec/dora_mac_aec/lib/'}\n"
                f"  3. {Path(__file__).parent / '../../node-hub/dora-aec/dora_aec/lib/'}"
            )
        
        self.aec = AECWrapper(
            library_path=str(lib_path),
            enable_aec=True,
            enable_vad=True,
            sample_rate=self.sample_rate
        )
        if self.node:
            send_log(self.node, "INFO", f"MAC-AEC initialized with library: {lib_path.name}")
        
    def start(self):
        """Start the MAC-AEC capture"""
        try:
            self.aec.start_record()
            
            # Test if we can get audio
            import time
            time.sleep(0.1)
            
            test_data, test_vad = self.aec.get_audio_data()
            if self.node:
                if test_data:
                    send_log(self.node, "INFO", "MAC-AEC started successfully")
                else:
                    send_log(self.node, "WARNING", "MAC-AEC started but no audio data in test")
            
            return True
        except Exception as e:
            if self.node:
                send_log(self.node, "ERROR", f"Failed to start MAC-AEC: {e}")
            raise
        
    def stop(self):
        """Stop the MAC-AEC capture"""
        self.aec.stop_record()
        if self.node:
            send_log(self.node, "INFO", "MAC-AEC stopped")
            
    def process_audio(self, node):
        """Process audio and detect speech segments"""
        try:
            # Collect ALL available audio frames (drain the buffer)
            all_audio = []
            vad_results = []
            
            # Keep getting audio until buffer is empty
            max_iterations = 100  # Safety limit
            iterations = 0
            
            while iterations < max_iterations:
                try:
                    audio_data, vad_result = self.aec.get_audio_data()
                except Exception as e:
                    if self.node:
                        send_log(self.node, "ERROR", f"Error getting audio: {e}")
                    break
                    
                iterations += 1
                
                if audio_data is None:
                    break
                    
                if isinstance(audio_data, bytes) and len(audio_data) > 0:
                    # Convert and collect
                    audio_array = np.frombuffer(audio_data, dtype=np.int16).astype(np.float32) / 32768.0
                    all_audio.append(audio_array)
                    vad_results.append(vad_result)
                    
                    # Debug: Show we got data
                    if self._debug_counter % 10 == 0 and self.node:
                        send_log(self.node, "DEBUG", f"Got chunk {len(all_audio)}: {len(audio_data)} bytes = {len(audio_array)} samples")
                elif audio_data is not None:
                    if self.node:
                        send_log(self.node, "WARNING", f"Got non-bytes data: {type(audio_data)}")
                    break
            
            # If no audio was available, return None
            if len(all_audio) == 0:
                return None, False, False, None
            
            # Combine all audio chunks
            audio_array = np.concatenate(all_audio) if len(all_audio) > 1 else all_audio[0]
            vad_result = any(vad_results)  # True if any chunk had voice
                
            # Debug: Check audio characteristics periodically
            if not hasattr(self, '_debug_counter'):
                self._debug_counter = 0
            self._debug_counter += 1
                
            if self._debug_counter % 30 == 0:  # Log every ~1 second
                max_val = np.abs(audio_array).max()
                rms = np.sqrt(np.mean(audio_array**2))
                chunks_collected = len(all_audio)
                if self.node:
                    send_log(self.node, "DEBUG", f"Collected {chunks_collected} chunks, total {len(audio_array)} samples, max={max_val:.4f}, rms={rms:.4f}, VAD={vad_result}")
                
            # Track VAD state changes
            speech_started = False
            speech_ended = False
            audio_segment = None
            
            # Update speech state based on VAD
            if vad_result:
                # Speech detected
                if not self.is_speaking:
                    self.silence_count = 0
                    self.speech_buffer.append(audio_array)
                    
                    # Check if we have enough speech to start
                    if len(self.speech_buffer) >= self.speech_start_threshold:
                        self.is_speaking = True
                        speech_started = True
                        # Speech started - no verbose logging
                        
                        # Start new segment buffer
                        self.audio_segment_buffer = []
                        for buf in self.speech_buffer:
                            self.audio_segment_buffer.extend(buf)
                else:
                    # Continue adding to segment
                    self.audio_segment_buffer.extend(audio_array)
                    self.silence_count = 0
                    
                    # Check max segment size
                    if len(self.audio_segment_buffer) >= self.max_segment_size:
                        # Force segment end at max size
                        audio_segment = np.array(self.audio_segment_buffer, dtype=np.float32)
                        self.audio_segment_buffer = []
                        self.is_speaking = False
                        speech_ended = True
                        # Max segment size reached - no verbose logging
                        
            else:
                # No speech detected
                if self.is_speaking:
                    # Add to buffer even during silence (for natural endings)
                    self.audio_segment_buffer.extend(audio_array)
                    self.silence_count += 1
                    
                    # Check if silence is long enough to end speech
                    if self.silence_count >= self.speech_end_threshold:
                        # Speech ended - create segment
                        if len(self.audio_segment_buffer) >= self.min_segment_size:
                            audio_segment = np.array(self.audio_segment_buffer, dtype=np.float32)
                            # Speech ended - no verbose logging
                        else:
                            if self.node:
                                send_log(self.node, "DEBUG", f"Segment too short, discarding: {len(self.audio_segment_buffer)} samples")
                            
                        # Reset state
                        self.audio_segment_buffer = []
                        self.is_speaking = False
                        self.silence_count = 0
                        self.speech_buffer = []
                        speech_ended = True
                else:
                    # Not speaking, clear buffers if any
                    if len(self.speech_buffer) > 0:
                        self.speech_buffer = []
                        
            return audio_array, speech_started, speech_ended, audio_segment
            
        except Exception as e:
            if self.node:
                send_log(self.node, "ERROR", f"Error processing audio: {e}")
            return None, False, False, None


def main():
    """Main node loop"""
    node = Node("mac-aec")
    
    send_log(node, "INFO", "MAC-AEC Simple Segmentation Node starting")
    send_log(node, "INFO", "Wrapping dora-mac-aec with VAD-based segmentation")
    
    # Initialize wrapper
    mac_aec = MacAECSegmentation(node)
    
    # Start MAC-AEC
    mac_aec.start()
        
    send_log(node, "INFO", "Node ready - outputting: audio, is_speaking, speech_started, speech_ended, audio_segment")
    
    # State tracking
    frame_count = 0
    last_audio_time = time.time()
    no_audio_count = 0
    
    try:
        while True:
            # Check for events with timeout
            event = node.next(timeout=0.1)  # 100ms timeout as required
            
            if event and event["type"] == "INPUT":
                # Handle control commands if needed
                input_id = event.get("id", "")
                if input_id == "control":
                    # Handle control command
                    data = event["value"][0].as_py()
                    send_log(node, "INFO", f"Received control command: {data}")
                
            # Process audio continuously
            current_time = time.time()
            
            # Process more frequently to avoid missing audio (every 10ms)
            if current_time - last_audio_time > 0.010:
                audio_frame, speech_started, speech_ended, audio_segment = mac_aec.process_audio(node)
                
                if audio_frame is not None:
                    # Send ALL audio frames for complete recording
                    frame_count += 1
                    # Always send audio for continuous recording
                    audio_pa = pa.array(audio_frame, type=pa.float32())
                    node.send_output("audio", audio_pa)
                    no_audio_count = 0
                    
                    # Log progress and audio details
                    if frame_count % 30 == 0:  # Every ~300ms
                        max_val = np.abs(audio_frame).max() if len(audio_frame) > 0 else 0
                        send_log(node, "DEBUG", f"Sent frame {frame_count}: {len(audio_frame)} samples, max_amp={max_val:.4f}")
                else:
                    # Track no audio frames
                    no_audio_count += 1
                    if no_audio_count > 100 and no_audio_count % 100 == 0:
                        send_log(node, "WARNING", f"No audio for {no_audio_count} frames")
                        
                # Send VAD state changes
                if speech_started:
                    node.send_output("speech_started", pa.array([current_time]))
                    node.send_output("is_speaking", pa.array([True]))
                    
                if speech_ended:
                    node.send_output("speech_ended", pa.array([current_time]))
                    node.send_output("is_speaking", pa.array([False]))
                    
                # Send audio segment when ready
                if audio_segment is not None:
                    # Segment sent - no verbose logging
                    # Send as float32 array for ASR compatibility
                    node.send_output("audio_segment", pa.array(audio_segment, type=pa.float32()))
                    
                last_audio_time = current_time
                
    except KeyboardInterrupt:
        send_log(node, "INFO", "Shutting down...")
    except Exception as e:
        send_log(node, "ERROR", f"Error in main loop: {e}")
        import traceback
        traceback.print_exc()
    finally:
        mac_aec.stop()
        send_log(node, "INFO", "Stopped")


if __name__ == "__main__":
    main()