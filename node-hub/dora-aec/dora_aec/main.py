#!/usr/bin/env python3
"""
Dora AEC (Acoustic Echo Cancellation) Node - Fixed version

This node provides acoustic echo cancellation and voice activity detection
using macOS's native VoiceProcessingIO AudioUnit.
"""

import os
import time
import threading
import queue
from typing import Optional
from pathlib import Path

import numpy as np
import pyarrow as pa
from dora import Node

from .aec_wrapper import AECWrapper


class Config:
    """Configuration for AEC node"""
    # Audio configuration
    SAMPLE_RATE = 16000
    CHANNELS = 1
    BUFFER_SIZE = 512
    OUTPUT_FORMAT = "int16"
    
    # Feature flags
    AEC_ENABLED = True
    VAD_ENABLED = True
    
    # Control
    AUTO_START = True  # Automatically start capture on node initialization
    
    # Timing
    POLL_INTERVAL_MS = 10  # How often to poll for audio data


class AECNode:
    """
    Acoustic Echo Cancellation node for Dora
    
    Processes microphone input with echo cancellation using TTS reference signal.
    """
    
    def __init__(self, node: Node):
        self.node = node
        self.config = Config()
        self.aec_wrapper: Optional[AECWrapper] = None
        self.is_running = False
        self.is_paused = False
        
        # Use queues for thread-safe communication
        self.microphone_queue = queue.Queue(maxsize=100)
        self.reference_queue = queue.Queue(maxsize=100)
        self.output_queue = queue.Queue(maxsize=100)
        
        # Processing thread
        self.process_thread: Optional[threading.Thread] = None
        
        # Load configuration from environment variables
        self._load_config()
        
        # Initialize AEC wrapper
        self._initialize_aec()
        
        # Auto-start if configured (with small delay for other nodes to initialize)
        if self.config.AUTO_START:
            import time
            time.sleep(2.0)  # Give other nodes time to initialize
            self.start_capture()
    
    def _load_config(self):
        """Load configuration from environment variables"""
        # Audio configuration
        self.config.SAMPLE_RATE = int(os.getenv("SAMPLE_RATE", str(self.config.SAMPLE_RATE)))
        self.config.CHANNELS = int(os.getenv("CHANNELS", str(self.config.CHANNELS)))
        self.config.BUFFER_SIZE = int(os.getenv("BUFFER_SIZE", str(self.config.BUFFER_SIZE)))
        self.config.OUTPUT_FORMAT = os.getenv("OUTPUT_FORMAT", self.config.OUTPUT_FORMAT)
        
        # Feature flags
        self.config.AEC_ENABLED = os.getenv("AEC_ENABLED", "true").lower() == "true"
        self.config.VAD_ENABLED = os.getenv("VAD_ENABLED", "true").lower() == "true"
        
        # Control
        self.config.AUTO_START = os.getenv("AUTO_START", "true").lower() == "true"
        
        # Timing
        self.config.POLL_INTERVAL_MS = int(os.getenv("POLL_INTERVAL_MS", str(self.config.POLL_INTERVAL_MS)))
        
        self._log_info(f"Configuration loaded: AEC={self.config.AEC_ENABLED}, VAD={self.config.VAD_ENABLED}, Sample Rate={self.config.SAMPLE_RATE}")
    
    def _initialize_aec(self):
        """Initialize the AEC wrapper"""
        try:
            # Find the native library
            lib_path = Path(__file__).parent / "lib" / "libAudioCapture.dylib"
            
            if lib_path.exists():
                self.aec_wrapper = AECWrapper(
                    library_path=str(lib_path),
                    enable_aec=self.config.AEC_ENABLED,
                    enable_vad=self.config.VAD_ENABLED,
                    sample_rate=self.config.SAMPLE_RATE
                )
                self._log_info(f"AEC wrapper initialized with library at {lib_path}")
            else:
                # Run in passthrough mode without native library
                self._log_warning(f"libAudioCapture.dylib not found at {lib_path}, running in passthrough mode")
                self.aec_wrapper = None
            
        except Exception as e:
            self._log_error(f"Failed to initialize AEC wrapper: {e}")
            # Run in passthrough mode on error
            self.aec_wrapper = None
            self._log_warning("Running in passthrough mode due to initialization error")
    
    def start_capture(self):
        """Start audio processing"""
        if self.is_running:
            self._log_warning("Processing already running")
            return
        
        self.is_running = True
        self.is_paused = False
        
        # Start capture in AEC wrapper if available
        if self.aec_wrapper:
            self.aec_wrapper.start_record()
        
        # Start processing thread
        self.process_thread = threading.Thread(target=self._process_loop, daemon=True)
        self.process_thread.start()
        
        self._log_info("AEC processing started" if self.aec_wrapper else "Passthrough mode started")
    
    def stop_capture(self):
        """Stop audio processing"""
        if not self.is_running:
            return
        
        self.is_running = False
        
        # Wait for processing thread to finish
        if self.process_thread:
            self.process_thread.join(timeout=1.0)
        
        # Stop capture in AEC wrapper if available
        if self.aec_wrapper:
            self.aec_wrapper.stop_record()
        
        self._log_info("AEC processing stopped" if self.aec_wrapper else "Passthrough mode stopped")
    
    def pause_capture(self):
        """Pause audio capture (audio is still captured but not sent)"""
        self.is_paused = True
        self._log_info("Audio capture paused")
    
    def resume_capture(self):
        """Resume audio capture"""
        self.is_paused = False
        self._log_info("Audio capture resumed")
    
    def _process_loop(self):
        """Main processing loop - combines microphone input with AEC"""
        poll_interval = self.config.POLL_INTERVAL_MS / 1000.0
        
        while self.is_running:
            try:
                if self.aec_wrapper:
                    # When AEC wrapper is active, it captures audio directly from hardware
                    # We don't need external microphone input - the wrapper IS the microphone
                    audio_data, vad_status = self.aec_wrapper.get_audio_data()
                    
                    if audio_data is not None and len(audio_data) > 0:
                        if not self.is_paused:
                            # Convert int16 bytes to float32 for Speech Monitor
                            # audio_data is raw bytes in int16 format
                            audio_array = np.frombuffer(audio_data, dtype=np.int16)
                            # Convert to float32 in range [-1, 1]
                            audio_float32 = audio_array.astype(np.float32) / 32768.0
                            
                            # Log diagnostics periodically
                            if hasattr(self, '_audio_count'):
                                self._audio_count += 1
                            else:
                                self._audio_count = 1
                            
                            if self._audio_count % 500 == 0:
                                max_val = np.abs(audio_float32).max()
                                self._log_info(f"Audio stats: {len(audio_float32)} samples, max amplitude: {max_val:.4f}, VAD: {vad_status}")
                            
                            # Queue output for sending with metadata
                            audio_pa = pa.array(audio_float32)
                            if self._audio_count % 100 == 0:
                                self._log_info(f"Queuing audio output #{self._audio_count}: {len(audio_float32)} samples, dtype: {audio_float32.dtype}")
                            self.output_queue.put(("audio", audio_pa))
                            if self.config.VAD_ENABLED:
                                self.output_queue.put(("vad_status", pa.array([vad_status])))
                    else:
                        # No audio available, wait briefly
                        time.sleep(poll_interval)
                else:
                    # Passthrough mode - use external microphone input
                    try:
                        mic_data = self.microphone_queue.get(timeout=poll_interval)
                        
                        if not self.is_paused:
                            self.output_queue.put(("audio", mic_data))
                            if self.config.VAD_ENABLED:
                                self.output_queue.put(("vad_status", pa.array([True])))
                                
                    except queue.Empty:
                        # No microphone data available
                        pass
                    
            except Exception as e:
                # Queue error message instead of sending directly
                try:
                    self.output_queue.put(("log", pa.array([f"[ERROR] AEC: Error in processing loop: {e}"])))
                except:
                    print(f"Error in processing loop: {e}")
                time.sleep(poll_interval)
    
    def handle_control(self, command: str, args: dict = None):
        """Handle control commands"""
        args = args or {}
        
        if command == "start":
            self.start_capture()
        elif command == "stop":
            self.stop_capture()
        elif command == "pause":
            self.pause_capture()
        elif command == "resume":
            self.resume_capture()
        elif command == "configure":
            self._handle_configure(args)
        else:
            self._log_warning(f"Unknown control command: {command}")
    
    def handle_microphone_input(self, audio_data):
        """Handle raw microphone input"""
        # Convert to PyArrow array if needed
        if not isinstance(audio_data, pa.Array):
            audio_data = pa.array(audio_data)
        
        try:
            self.microphone_queue.put_nowait(audio_data)
        except queue.Full:
            # Drop oldest if queue is full
            try:
                self.microphone_queue.get_nowait()
                self.microphone_queue.put_nowait(audio_data)
            except:
                pass
    
    def handle_reference_audio(self, audio_data):
        """Handle reference audio from TTS for echo cancellation"""
        # Store reference audio for AEC processing
        try:
            self.reference_queue.put_nowait(audio_data)
        except queue.Full:
            # Drop oldest if queue is full
            try:
                self.reference_queue.get_nowait()
                self.reference_queue.put_nowait(audio_data)
            except:
                pass
    
    def _handle_configure(self, args: dict):
        """Handle configuration changes"""
        # Update configuration
        if "AEC_ENABLED" in args:
            self.config.AEC_ENABLED = args["AEC_ENABLED"]
        if "VAD_ENABLED" in args:
            self.config.VAD_ENABLED = args["VAD_ENABLED"]
        
        # Restart with new configuration
        was_running = self.is_running
        if was_running:
            self.stop_capture()
        
        self._initialize_aec()
        
        if was_running:
            self.start_capture()
        
        self._log_info(f"Configuration updated: {args}")
    
    def _log_info(self, message: str):
        """Queue info log message"""
        try:
            self.output_queue.put(("log", pa.array([f"[INFO ] AEC: {message}"])))
        except:
            print(f"[INFO ] AEC: {message}")
    
    def _log_warning(self, message: str):
        """Queue warning log message"""
        try:
            self.output_queue.put(("log", pa.array([f"[WARN ] AEC: {message}"])))
        except:
            print(f"[WARN ] AEC: {message}")
    
    def _log_error(self, message: str):
        """Queue error log message"""
        try:
            self.output_queue.put(("log", pa.array([f"[ERROR] AEC: {message}"])))
        except:
            print(f"[ERROR] AEC: {message}")
    
    def _log_debug(self, message: str):
        """Queue debug log message"""
        try:
            self.output_queue.put(("log", pa.array([f"[DEBUG] AEC: {message}"])))
        except:
            print(f"[DEBUG] AEC: {message}")
    
    def cleanup(self):
        """Clean up resources"""
        self.stop_capture()
        if self.aec_wrapper:
            self.aec_wrapper.cleanup()


def main():
    """Main entry point for Dora AEC node"""
    print("[AEC MAIN] Starting AEC node...")
    node = Node()
    print("[AEC MAIN] Node created")
    aec_node = AECNode(node)
    print("[AEC MAIN] AECNode initialized")
    
    output_count = 0
    last_log_time = time.time()
    
    try:
        while True:
            # Process any queued outputs
            outputs_sent = 0
            try:
                while not aec_node.output_queue.empty():
                    output_id, data = aec_node.output_queue.get_nowait()
                    node.send_output(output_id, data)
                    outputs_sent += 1
                    
                    if output_id == "audio":
                        output_count += 1
                        # Log every output for debugging
                        audio_len = len(data.to_numpy()) if hasattr(data, 'to_numpy') else len(data)
                        if output_count <= 10 or output_count % 100 == 0:
                            print(f"[AEC MAIN] Sent audio output #{output_count}: {audio_len} samples to Speech Monitor via node.send_output()")
                            print(f"[AEC MAIN] Data type: {type(data)}, PA array: {data}")
                    elif output_id == "log":
                        # Print logs directly too
                        log_msg = data[0].as_py() if hasattr(data[0], 'as_py') else str(data[0])
                        print(f"[AEC LOG] {log_msg}")
            except queue.Empty:
                pass
            except Exception as e:
                print(f"Error sending output: {e}")
            
            # Periodic status log
            if time.time() - last_log_time > 5.0:
                print(f"[AEC MAIN] Status: {output_count} audio outputs sent, queue size: {aec_node.output_queue.qsize()}")
                last_log_time = time.time()
            
            # Process input events
            event = node.next(timeout=0.01)  # Short timeout to check outputs frequently
            
            if event is None:
                continue
            
            if event["type"] == "INPUT":
                input_id = event.get("id", "")
                
                if input_id == "microphone":
                    # Handle raw microphone input
                    audio_data = event["value"].to_numpy()
                    aec_node.handle_microphone_input(audio_data)
                
                elif input_id == "control":
                    # Handle control commands
                    data = event["value"][0].as_py()
                    if isinstance(data, dict):
                        command = data.get("command", "")
                        args = data.get("args", {})
                        aec_node.handle_control(command, args)
                    elif isinstance(data, str):
                        aec_node.handle_control(data)
                
                elif input_id == "reference_audio":
                    # Handle reference audio from TTS for echo cancellation
                    audio_data = event["value"].to_numpy()
                    aec_node.handle_reference_audio(audio_data)
    
    except KeyboardInterrupt:
        print("Received interrupt signal")
    except Exception as e:
        print(f"Unexpected error: {e}")
    finally:
        aec_node.cleanup()


if __name__ == "__main__":
    main()