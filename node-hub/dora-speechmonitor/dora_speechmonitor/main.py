"""
Dora Speech Monitor Node
Real-time speech monitoring with state tracking, based on VoiceDialogue.
"""

import time
import json
import numpy as np
import pyarrow as pa
from dora import Node

from .config import SpeechMonitorConfig
from .vad import SileroVAD
from .state_machine import SpeechStateMachine, VoiceTask, SpeechState


def calculate_audio_duration(audio_data: np.ndarray, sample_rate: int) -> float:
    """Calculate audio duration in seconds"""
    return len(audio_data) / sample_rate


def normalize_audio_frame(data: bytes) -> np.ndarray:
    """Convert int16 audio bytes to float32 numpy array [-1.0, 1.0]"""
    audio_array = np.frombuffer(data, dtype=np.int16)
    return audio_array.astype(np.float32) / 32768.0


def send_log(node, level, message):
    """Send log message through log output channel.
    
    Args:
        node: Dora node instance
        level: Log level (DEBUG, INFO, WARNING, ERROR)
        message: Log message
    """
    # Define log level hierarchy
    LOG_LEVELS = {
        "DEBUG": 10,
        "INFO": 20,
        "WARNING": 30,
        "ERROR": 40
    }
    
    # Get configured log level
    from .config import SpeechMonitorConfig
    config_level = SpeechMonitorConfig.LOG_LEVEL
    
    # Check if message should be logged
    if LOG_LEVELS.get(level, 0) < LOG_LEVELS.get(config_level, 20):
        return  # Skip messages below configured level
    
    # Format message with level prefix
    formatted_message = f"[{level}] {message}"
    
    log_data = {
        "node": "speech-monitor",
        "level": level,
        "message": formatted_message,
        "timestamp": time.time()
    }
    node.send_output("log", pa.array([json.dumps(log_data)]))


def main():
    """Main entry point for speech monitor node"""
    
    # Initialize components
    node = Node()
    config = SpeechMonitorConfig()
    state_machine = SpeechStateMachine()
    
    # Initialize VAD if enabled
    vad_instance = None
    if config.VAD_ENABLED:
        vad_instance = SileroVAD(threshold=config.VAD_THRESHOLD)
    
    # Audio buffer
    audio_frames = np.array([])
    
    # Pre-speech buffer for capturing speech onset
    pre_speech_buffer = np.array([])
    pre_speech_buffer_size = int(0.2 * config.SAMPLE_RATE)  # 200ms
    
    send_log(node, "INFO", "Speech Monitor initialized")
    send_log(node, "INFO", f"VAD: {'Enabled' if config.VAD_ENABLED else 'Disabled'}")
    send_log(node, "INFO", f"Sample rate: {config.SAMPLE_RATE} Hz")
    send_log(node, "INFO", f"Log level: {config.LOG_LEVEL}")
    send_log(node, "DEBUG", f"Silence threshold: {config.SILENCE_THRESHOLD}ms")
    send_log(node, "DEBUG", f"User silence threshold: {config.USER_SILENCE_THRESHOLD}ms")
    
    # Event tracking
    last_speech_start_time = None
    speech_segment_count = 0
    last_speech_end_time = None  # Track when speech last ended
    question_end_sent = False  # Track if question_ended was sent for current silence period
    
    # Pause/Resume control
    is_paused = False
    
    for event in node:
        # Handle control signals for pause/resume
        if event["type"] == "INPUT" and event["id"] == "control":
            control_cmd = event["value"][0].as_py()
            
            if control_cmd == "pause":
                if not is_paused:
                    is_paused = True
                    send_log(node, "INFO", "Speech Monitor PAUSED")
                    # Reset state when pausing
                    state_machine.reset()
                    audio_frames = np.array([])
                    pre_speech_buffer = np.array([])
                    last_speech_end_time = None  # Reset timing
                    question_end_sent = False
            
            elif control_cmd == "resume":
                if is_paused:
                    is_paused = False
                    send_log(node, "INFO", "Speech Monitor RESUMED")
                    # Reset state for fresh start
                    state_machine.reset()
                    audio_frames = np.array([])
                    pre_speech_buffer = np.array([])
                    speech_segment_count = 0
                    last_speech_end_time = None  # Reset timing
                    question_end_sent = False
            
            elif control_cmd == "status":
                status_msg = f"Paused: {is_paused}, State: {state_machine.state.name}"
                send_log(node, "INFO", status_msg)
            
            continue
        
        if event["type"] == "INPUT" and event["id"] == "audio":
            # Get audio chunk
            audio_chunk = event["value"].to_numpy()
            sr = event.get("metadata", {}).get("sample_rate", config.SAMPLE_RATE)
            
            # Log received audio periodically (every 10 chunks for better feedback)
            if not hasattr(state_machine, '_audio_receive_count'):
                state_machine._audio_receive_count = 0
            state_machine._audio_receive_count += 1
            
            if state_machine._audio_receive_count % 10 == 0:
                max_amp = np.abs(audio_chunk).max() if len(audio_chunk) > 0 else 0
                send_log(node, "DEBUG", f"Received audio chunk #{state_machine._audio_receive_count}: {len(audio_chunk)} samples, max amp: {max_amp:.4f}")
            
            # Drop all audio if paused
            if is_paused:
                # Simply discard the audio chunk and continue
                continue
            
            # Calculate chunk duration
            chunk_duration_ms = calculate_audio_duration(audio_chunk, sr) * 1000
            
            # Detect speech activity
            is_voice_active = False
            speech_probability = 0.0
            
            if config.VAD_ENABLED and vad_instance:
                is_voice_active, speech_probability = vad_instance.is_voice_active(audio_chunk, sr)
            else:
                # Simple amplitude-based detection as fallback
                max_amplitude = np.abs(audio_chunk).max()
                is_voice_active = max_amplitude > config.MIN_AUDIO_AMPLITUDE
                speech_probability = min(max_amplitude * 10, 1.0)
            
            # Send real-time speech probability
            node.send_output(
                "speech_probability",
                pa.array([speech_probability]),
                metadata={"timestamp": time.time()}
            )
            
            # Send is_speaking status
            node.send_output(
                "is_speaking",
                pa.array([state_machine.state == SpeechState.SPEAKING]),
                metadata={"timestamp": time.time()}
            )
            
            # State machine processing
            if is_voice_active:
                # Speech detected
                if np.max(np.abs(audio_chunk)) <= config.MIN_AUDIO_AMPLITUDE:
                    # Too quiet, ignore
                    continue
                    
                # Handle state transition
                if state_machine.state == SpeechState.SILENCE:
                    # Speech started
                    state_machine.transition_to_speaking()
                    last_speech_start_time = time.time()
                    question_end_sent = False  # Reset when new speech starts
                    speech_segment_count += 1
                    
                    # Include pre-speech buffer
                    if len(pre_speech_buffer) > 0:
                        audio_frames = pre_speech_buffer.copy()
                    else:
                        audio_frames = np.array([])
                    
                    # Send speech_started event
                    node.send_output(
                        "speech_started",
                        pa.array([last_speech_start_time]),
                        metadata={
                            "task_id": state_machine.task_id,
                            "segment": speech_segment_count
                        }
                    )
                    send_log(node, "INFO", f"Speech STARTED (segment #{speech_segment_count})")
                    
                elif state_machine.state == SpeechState.TRAILING_SILENCE:
                    # Speech resumed
                    state_machine.transition_to_speaking()
                    send_log(node, "DEBUG", "Speech RESUMED")
                
                # Update duration tracking
                state_machine.update_active_duration(chunk_duration_ms)
                state_machine.user_silence_duration = 0
                
                # Append to buffer
                audio_frames = np.append(audio_frames, audio_chunk)
                state_machine.is_audio_frames_empty = False
                
                # Check for interrupt condition (from VoiceDialogue)
                if state_machine.active_audio_frame_duration > config.ACTIVE_FRAME_THRESHOLD:
                    # Speech is continuing, could trigger interrupt in full system
                    pass
                
            else:
                # Silence detected
                state_machine.update_silence_duration(chunk_duration_ms)
                
                if state_machine.state == SpeechState.SPEAKING:
                    # Transition to trailing silence
                    state_machine.transition_to_trailing_silence()
                    send_log(node, "DEBUG", "Trailing silence...")
                    
                    # Still append audio (might resume)
                    audio_frames = np.append(audio_frames, audio_chunk)
                    
                elif state_machine.state == SpeechState.TRAILING_SILENCE:
                    # Continue trailing silence
                    audio_frames = np.append(audio_frames, audio_chunk)
                    
                    # Check if silence is long enough to end speech
                    if state_machine.is_user_in_silence(config.SILENCE_THRESHOLD):
                        # Speech ended
                        speech_end_time = time.time()
                        last_speech_end_time = speech_end_time  # Track for question_ended detection
                        question_end_sent = False  # Reset flag for new silence period
                        
                        # Send speech_ended event
                        node.send_output(
                            "speech_ended",
                            pa.array([speech_end_time]),
                            metadata={
                                "task_id": state_machine.task_id,
                                "segment": speech_segment_count,
                                "duration": speech_end_time - last_speech_start_time if last_speech_start_time else 0
                            }
                        )
                        
                        # Send complete audio segment
                        if len(audio_frames) > 0:
                            # Check if over threshold
                            audio_duration_ms = calculate_audio_duration(audio_frames, sr) * 1000
                            is_over_threshold = audio_duration_ms >= config.AUDIO_FRAMES_THRESHOLD
                            
                            # Create voice task
                            voice_task = VoiceTask.create(
                                task_id=state_machine.task_id,
                                session_id=state_machine.session_id,
                                audio_data=audio_frames
                            )
                            voice_task.is_over_audio_frames_threshold = is_over_threshold
                            
                            # Send audio segment
                            node.send_output(
                                "audio_segment",
                                pa.array(audio_frames),
                                metadata={
                                    "task_id": state_machine.task_id,
                                    "session_id": state_machine.session_id,
                                    "answer_id": voice_task.answer_id,
                                    "duration_ms": audio_duration_ms,
                                    "is_over_threshold": is_over_threshold,
                                    "segment": speech_segment_count
                                }
                            )
                            
                            duration_s = audio_duration_ms / 1000
                            send_log(node, "INFO", f"Speech SEGMENT sent: {duration_s:.2f}s")
                        
                        # Transition to silence
                        state_machine.transition_to_silence()
                        audio_frames = np.array([])
                        
                    # Check for user silence (longer threshold)
                    if state_machine.is_user_in_silence(config.USER_SILENCE_THRESHOLD):
                        # User has been silent for a while
                        # In full system, this would trigger silence_over_threshold_event
                        pass
                        
                elif state_machine.state == SpeechState.SILENCE:
                    # Maintain pre-speech buffer
                    pre_speech_buffer = np.append(pre_speech_buffer, audio_chunk)
                    
                    # Keep only last 200ms
                    if len(pre_speech_buffer) > pre_speech_buffer_size:
                        pre_speech_buffer = pre_speech_buffer[-pre_speech_buffer_size:]
                    
                    # Check for question_ended signal (longer silence after speech)
                    if last_speech_end_time and not question_end_sent:
                        silence_since_speech_ms = (time.time() - last_speech_end_time) * 1000
                        if silence_since_speech_ms >= config.QUESTION_END_SILENCE_THRESHOLD:
                            # Long silence detected - user question is complete
                            node.send_output(
                                "question_ended",
                                pa.array([time.time()]),
                                metadata={
                                    "silence_duration_ms": silence_since_speech_ms,
                                    "last_segment": speech_segment_count
                                }
                            )
                            send_log(node, "INFO", f"Question ENDED (silence: {silence_since_speech_ms:.0f}ms)")
                            question_end_sent = True  # Prevent repeated signals
            
            # Check for max segment duration
            if len(audio_frames) > 0:
                current_duration_ms = calculate_audio_duration(audio_frames, sr) * 1000
                if current_duration_ms >= config.AUDIO_FRAMES_THRESHOLD:
                    # Force segment end due to length
                    send_log(node, "WARNING", "Max segment duration reached, forcing segment end")
                    
                    # Send audio segment
                    node.send_output(
                        "audio_segment",
                        pa.array(audio_frames),
                        metadata={
                            "task_id": state_machine.task_id,
                            "forced": True,
                            "duration_ms": current_duration_ms,
                            "segment": speech_segment_count
                        }
                    )
                    
                    # Reset buffers
                    audio_frames = np.array([])
                    state_machine.reset()


if __name__ == "__main__":
    main()