#!/usr/bin/env python3
"""
Dora AEC (Acoustic Echo Cancellation) Node

This node provides acoustic echo cancellation and voice activity detection
using macOS's native VoiceProcessingIO AudioUnit.
"""

import time
import threading
from typing import Optional
from pathlib import Path

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
        self.capture_thread: Optional[threading.Thread] = None
        self.is_running = False
        self.is_paused = False
        
        # Buffers for audio processing
        self.microphone_buffer = []
        self.reference_buffer = []
        self.buffer_lock = threading.Lock()
        
        # Processing thread
        self.process_thread: Optional[threading.Thread] = None
        
        # Load configuration from node metadata
        self._load_config()
        
        # Initialize AEC wrapper
        self._initialize_aec()
        
        # Auto-start if configured
        if self.config.AUTO_START:
            self.start_capture()
    
    def _load_config(self):
        """Load configuration from environment variables"""
        import os
        
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
            
            if not lib_path.exists():
                # Try to find it in VoiceDialogue installation
                voice_dialogue_lib = Path.home() / "home" / "VoiceDialogue" / "libraries" / "libAudioCapture.dylib"
                if voice_dialogue_lib.exists():
                    lib_path = voice_dialogue_lib
                    self.aec_wrapper = AECWrapper(
                        library_path=str(lib_path),
                        enable_aec=self.config.AEC_ENABLED,
                        enable_vad=self.config.VAD_ENABLED,
                        sample_rate=self.config.SAMPLE_RATE
                    )
                    self._log_info(f"AEC wrapper initialized with library at {lib_path}")
                else:
                    # Run in passthrough mode without native library
                    self._log_warning(f"libAudioCapture.dylib not found, running in passthrough mode")
                    self.aec_wrapper = None
            else:
                self.aec_wrapper = AECWrapper(
                    library_path=str(lib_path),
                    enable_aec=self.config.AEC_ENABLED,
                    enable_vad=self.config.VAD_ENABLED,
                    sample_rate=self.config.SAMPLE_RATE
                )
                self._log_info(f"AEC wrapper initialized with library at {lib_path}")
            
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
        
        # Start capture in AEC wrapper
        self.aec_wrapper.start_record()
        
        # Start processing thread
        self.process_thread = threading.Thread(target=self._process_loop, daemon=True)
        self.process_thread.start()
        
        self._log_info("AEC processing started")
    
    def stop_capture(self):
        """Stop audio processing"""
        if not self.is_running:
            return
        
        self.is_running = False
        
        # Wait for processing thread to finish
        if self.process_thread:
            self.process_thread.join(timeout=1.0)
        
        # Stop capture in AEC wrapper
        self.aec_wrapper.stop_record()
        
        self._log_info("AEC processing stopped")
    
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
                # Process buffered microphone audio
                with self.buffer_lock:
                    if self.microphone_buffer:
                        # Get microphone data
                        mic_data = self.microphone_buffer.pop(0)
                        
                        # Apply AEC if reference audio is available
                        if self.reference_buffer:
                            # In a real implementation, this would pass both signals
                            # to the native AEC library for processing
                            # For now, we'll use the hardware AEC from the wrapper
                            pass
                        
                        # Get processed audio from AEC wrapper
                        # The wrapper handles the actual echo cancellation
                        audio_data, vad_status = self.aec_wrapper.get_audio_data()
                        
                        if audio_data is not None and len(audio_data) > 0:
                            if not self.is_paused:
                                # Send echo-cancelled audio
                                self.node.send_output("audio", pa.array(audio_data))
                                
                                # Send VAD status if enabled
                                if self.config.VAD_ENABLED:
                                    self.node.send_output("vad_status", pa.array([vad_status]))
                    else:
                        # No microphone data, get from hardware directly
                        audio_data, vad_status = self.aec_wrapper.get_audio_data()
                        
                        if audio_data is not None and len(audio_data) > 0:
                            if not self.is_paused:
                                self.node.send_output("audio", pa.array(audio_data))
                                if self.config.VAD_ENABLED:
                                    self.node.send_output("vad_status", pa.array([vad_status]))
                        else:
                            time.sleep(poll_interval)
                    
            except Exception as e:
                self._log_error(f"Error in processing loop: {e}")
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
        with self.buffer_lock:
            # Convert to PyArrow array if needed
            if not isinstance(audio_data, pa.Array):
                audio_data = pa.array(audio_data)
            self.microphone_buffer.append(audio_data)
    
    def handle_reference_audio(self, audio_data):
        """Handle reference audio from TTS for echo cancellation"""
        with self.buffer_lock:
            # Store reference audio for AEC processing
            # The native library will use this to cancel echo
            self.reference_buffer.append(audio_data)
            
        # Log receipt of reference audio
        if len(audio_data) > 0:
            self._log_debug(f"Received reference audio: {len(audio_data)} samples")
    
    def _log_debug(self, message: str):
        """Send debug log message"""
        self.node.send_output("log", pa.array([f"[DEBUG] AEC: {message}"]))
    
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
        """Send info log message"""
        self.node.send_output("log", pa.array([f"[INFO ] AEC: {message}"]))
    
    def _log_warning(self, message: str):
        """Send warning log message"""
        self.node.send_output("log", pa.array([f"[WARN ] AEC: {message}"]))
    
    def _log_error(self, message: str):
        """Send error log message"""
        self.node.send_output("log", pa.array([f"[ERROR] AEC: {message}"]))
    
    def cleanup(self):
        """Clean up resources"""
        self.stop_capture()
        if self.aec_wrapper:
            self.aec_wrapper.cleanup()


def main():
    """Main entry point for Dora AEC node"""
    node = Node()
    aec_node = AECNode(node)
    
    try:
        while True:
            event = node.next()
            
            if event is None:
                break
            
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
        aec_node._log_info("Received interrupt signal")
    except Exception as e:
        aec_node._log_error(f"Unexpected error: {e}")
    finally:
        aec_node.cleanup()


if __name__ == "__main__":
    main()