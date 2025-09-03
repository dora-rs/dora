#!/usr/bin/env python3
"""
macOS AEC Dora Node - Main implementation
Provides echo-cancelled audio capture with comprehensive debugging
"""

import os
import time
import json
from pathlib import Path
from typing import Optional, Dict, Any

import numpy as np
import pyarrow as pa
from dora import Node

from .aec_wrapper import AECWrapper
from .utils.logger import AECLogger
from .utils.metrics import AudioMetrics


class MacAECNode:
    """
    macOS Acoustic Echo Cancellation Node
    
    Features:
    - Hardware-accelerated echo cancellation via VoiceProcessingIO
    - Voice Activity Detection (VAD)
    - Comprehensive logging and metrics
    - Dynamic configuration
    - Compatible with speech-monitor node
    """
    
    def __init__(self, node: Node):
        self.node = node
        self.logger = AECLogger()
        self.metrics = AudioMetrics()
        
        # Configuration
        self.config = self._load_config()
        
        # AEC wrapper
        self.aec: Optional[AECWrapper] = None
        self.is_active = False
        
        # Statistics
        self.frame_count = 0
        self.vad_positive_count = 0
        self.last_status_time = time.time()
        
        # Initialize AEC
        self._initialize_aec()
    
    def _load_config(self) -> Dict[str, Any]:
        """Load configuration from environment variables"""
        config = {
            # Audio settings
            'sample_rate': int(os.getenv('SAMPLE_RATE', '16000')),
            'chunk_size': int(os.getenv('CHUNK_SIZE', '512')),
            
            # Features
            'enable_aec': os.getenv('ENABLE_AEC', 'true').lower() == 'true',
            'enable_vad': os.getenv('ENABLE_VAD', 'true').lower() == 'true',
            
            # Logging
            'log_level': os.getenv('LOG_LEVEL', 'INFO'),
            'log_interval': float(os.getenv('LOG_INTERVAL', '5.0')),
            'enable_metrics': os.getenv('ENABLE_METRICS', 'true').lower() == 'true',
            
            # Debug
            'debug_audio': os.getenv('DEBUG_AUDIO', 'false').lower() == 'true',
            'save_audio': os.getenv('SAVE_AUDIO', 'false').lower() == 'true',
            'audio_dir': os.getenv('AUDIO_DIR', './debug_audio'),
            
            # Library path
            'lib_path': os.getenv('AEC_LIB_PATH', str(Path(__file__).parent / 'lib' / 'libAudioCapture.dylib'))
        }
        
        self.logger.info(f"Configuration loaded: {json.dumps(config, indent=2)}")
        return config
    
    def _initialize_aec(self):
        """Initialize AEC wrapper"""
        try:
            lib_path = Path(self.config['lib_path'])
            
            # Copy library from dora-aec if not exists
            if not lib_path.exists():
                src_lib = Path('/Users/yuechen/home/conversation/dora/node-hub/dora-aec/dora_aec/lib/libAudioCapture.dylib')
                if src_lib.exists():
                    lib_path.parent.mkdir(parents=True, exist_ok=True)
                    import shutil
                    shutil.copy2(src_lib, lib_path)
                    self.logger.info(f"Copied AEC library from {src_lib}")
            
            if not lib_path.exists():
                self.logger.error(f"AEC library not found at {lib_path}")
                self.aec = None
                return
            
            # Create wrapper
            self.aec = AECWrapper(
                library_path=str(lib_path),
                enable_aec=self.config['enable_aec'],
                enable_vad=self.config['enable_vad'],
                sample_rate=self.config['sample_rate']
            )
            
            self.logger.info("AEC wrapper initialized successfully")
            
        except Exception as e:
            self.logger.error(f"Failed to initialize AEC: {e}")
            self.aec = None
    
    def start(self):
        """Start audio capture"""
        if not self.aec:
            self.logger.error("Cannot start: AEC not initialized")
            return False
        
        if self.is_active:
            self.logger.warning("Already active")
            return True
        
        try:
            self.aec.start_record()
            self.is_active = True
            self.logger.info("Audio capture started")
            
            # Send status
            self._send_status("started")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to start capture: {e}")
            return False
    
    def stop(self):
        """Stop audio capture"""
        if not self.is_active:
            return
        
        if self.aec:
            self.aec.stop_record()
        
        self.is_active = False
        self.logger.info("Audio capture stopped")
        
        # Send final metrics
        if self.config['enable_metrics']:
            self._send_metrics()
        
        # Send status
        self._send_status("stopped")
    
    def process_audio(self) -> bool:
        """Process one audio chunk"""
        if not self.aec or not self.is_active:
            return False
        
        # Get audio from AEC
        audio_bytes, vad_status = self.aec.get_audio_data()
        
        
        if audio_bytes is None:
            # No audio available yet, this is normal
            return False
        
        # Convert to float32
        audio_int16 = np.frombuffer(audio_bytes, dtype=np.int16)
        audio_float32 = audio_int16.astype(np.float32) / 32768.0
        
        # Amplify audio if it's too quiet (common issue with AEC)
        max_amp = np.abs(audio_float32).max()
        if max_amp > 0 and max_amp < 0.01:
            # Amplify to a reasonable level
            amplification = 0.1 / max_amp
            audio_float32 = audio_float32 * amplification
            if self.frame_count % 100 == 0:
                print(f"[MAC-AEC] Amplified audio by {amplification:.1f}x (was {max_amp:.6f})")
        
        # Debug: Check audio amplitude every 100 frames
        if self.frame_count % 100 == 0:
            max_amp = np.abs(audio_float32).max()
            rms = np.sqrt(np.mean(audio_float32**2))
            print(f"[MAC-AEC] Audio check: max_amp={max_amp:.6f}, rms={rms:.6f}, samples={len(audio_float32)}")
        
        # Update metrics
        self.frame_count += 1
        if vad_status:
            self.vad_positive_count += 1
        
        if self.config['enable_metrics']:
            self.metrics.update(audio_float32, vad_status)
        
        # Debug logging
        if self.config['debug_audio'] and self.frame_count % 100 == 0:
            self._log_audio_debug(audio_float32, vad_status)
        
        # Save audio if enabled
        if self.config['save_audio']:
            self._save_audio_chunk(audio_float32, vad_status)
        
        # Send outputs
        self._send_audio(audio_float32)
        self._send_vad(vad_status)
        
        # Periodic status update
        if time.time() - self.last_status_time > self.config['log_interval']:
            self._send_periodic_status()
            self.last_status_time = time.time()
        
        return True
    
    def _send_audio(self, audio: np.ndarray):
        """Send audio output"""
        try:
            # Ensure audio is float32 and properly shaped
            if audio.dtype != np.float32:
                audio = audio.astype(np.float32)
            
            # Debug every 100th send
            if self.frame_count % 100 == 0:
                print(f"[MAC-AEC] Sending audio: shape={audio.shape}, dtype={audio.dtype}, max={np.abs(audio).max():.6f}")
            
            # Send as float32 array for speech-monitor compatibility
            audio_pa = pa.array(audio, type=pa.float32())
            self.node.send_output("audio", audio_pa)
        except Exception as e:
            self.logger.error(f"Failed to send audio: {e}")
    
    def _send_vad(self, vad_status: bool):
        """Send VAD status"""
        try:
            vad_pa = pa.array([vad_status])
            self.node.send_output("vad", vad_pa)
        except Exception as e:
            self.logger.error(f"Failed to send VAD: {e}")
    
    def _send_status(self, status: str, details: Dict = None):
        """Send status update"""
        try:
            status_data = {
                "status": status,
                "timestamp": time.time(),
                "frame_count": self.frame_count,
                "details": details or {}
            }
            status_pa = pa.array([json.dumps(status_data)])
            self.node.send_output("status", status_pa)
        except Exception as e:
            self.logger.error(f"Failed to send status: {e}")
    
    def _send_metrics(self):
        """Send metrics data"""
        try:
            metrics_data = self.metrics.get_summary()
            metrics_data['frame_count'] = self.frame_count
            metrics_data['vad_positive_rate'] = self.vad_positive_count / max(self.frame_count, 1)
            
            metrics_pa = pa.array([json.dumps(metrics_data)])
            self.node.send_output("metrics", metrics_pa)
            
            self.logger.info(f"Metrics: {json.dumps(metrics_data, indent=2)}")
        except Exception as e:
            self.logger.error(f"Failed to send metrics: {e}")
    
    def _send_periodic_status(self):
        """Send periodic status update"""
        vad_rate = (self.vad_positive_count / max(self.frame_count, 1)) * 100
        
        status_msg = (
            f"Frames: {self.frame_count}, "
            f"VAD: {vad_rate:.1f}%, "
            f"RMS: {self.metrics.current_rms:.4f}"
        )
        
        self.logger.info(status_msg)
        
        if self.config['enable_metrics']:
            self._send_metrics()
    
    def _log_audio_debug(self, audio: np.ndarray, vad: bool):
        """Log detailed audio debug info"""
        max_val = np.abs(audio).max()
        rms = np.sqrt(np.mean(audio**2))
        
        self.logger.debug(
            f"Audio[{self.frame_count}]: "
            f"samples={len(audio)}, "
            f"max={max_val:.4f}, "
            f"rms={rms:.4f}, "
            f"vad={vad}"
        )
    
    def _save_audio_chunk(self, audio: np.ndarray, vad: bool):
        """Save audio chunk for debugging"""
        if not self.config['save_audio']:
            return
        
        # Create directory
        audio_dir = Path(self.config['audio_dir'])
        audio_dir.mkdir(parents=True, exist_ok=True)
        
        # Save as numpy file
        filename = audio_dir / f"chunk_{self.frame_count:06d}_vad{int(vad)}.npy"
        np.save(filename, audio)
    
    def handle_control(self, command: str, args: Dict = None):
        """Handle control commands"""
        args = args or {}
        
        self.logger.info(f"Control command: {command}, args: {args}")
        
        if command == "start":
            self.start()
        elif command == "stop":
            self.stop()
        elif command == "configure":
            self._handle_configure(args)
        elif command == "get_status":
            self._send_current_status()
        elif command == "reset_metrics":
            self.metrics.reset()
            self.frame_count = 0
            self.vad_positive_count = 0
        else:
            self.logger.warning(f"Unknown command: {command}")
    
    def _handle_configure(self, args: Dict):
        """Handle configuration update"""
        # Update config
        for key, value in args.items():
            if key in self.config:
                self.config[key] = value
                self.logger.info(f"Config updated: {key} = {value}")
        
        # Restart if needed
        if self.is_active:
            self.stop()
            self._initialize_aec()
            self.start()
    
    def _send_current_status(self):
        """Send current detailed status"""
        details = {
            "is_active": self.is_active,
            "config": self.config,
            "metrics": self.metrics.get_summary() if self.config['enable_metrics'] else {},
            "frame_count": self.frame_count,
            "vad_positive_count": self.vad_positive_count
        }
        self._send_status("current", details)
    
    def cleanup(self):
        """Clean up resources"""
        self.stop()
        if self.aec:
            self.aec.cleanup()
        self.logger.info("Cleanup completed")


def main():
    """Main entry point for Dora node"""
    print("[MAC-AEC] Initializing macOS AEC node...")
    
    node = Node()
    aec_node = MacAECNode(node)
    
    # Auto-start if configured
    if os.getenv('AUTO_START', 'true').lower() == 'true':
        print("[MAC-AEC] Auto-starting audio capture...")
        aec_node.start()
        print("[MAC-AEC] Audio capture started, entering main loop...")
    
    # Add debug counter
    loop_count = 0
    audio_sent_count = 0
    
    print("[MAC-AEC] Starting main event loop...")
    try:
        while True:
            # Process audio
            got_audio = aec_node.process_audio()
            
            loop_count += 1
            if got_audio:
                audio_sent_count += 1
            
            
            # Handle events with longer timeout to give audio time to accumulate
            # 0.3s = 300ms gives the native library enough time between polls
            event = node.next(timeout=0.3)
            
            if event and event["type"] == "INPUT":
                input_id = event.get("id", "")
                
                if input_id == "control":
                    # Handle control command
                    data = event["value"][0].as_py()
                    if isinstance(data, dict):
                        command = data.get("command", "")
                        args = data.get("args", {})
                        aec_node.handle_control(command, args)
                    elif isinstance(data, str):
                        aec_node.handle_control(data)
    
    except KeyboardInterrupt:
        print("[MAC-AEC] Interrupted")
    except Exception as e:
        print(f"[MAC-AEC] Error: {e}")
    finally:
        aec_node.cleanup()


if __name__ == "__main__":
    main()