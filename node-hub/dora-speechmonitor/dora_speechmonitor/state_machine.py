"""
Speech state machine for tracking speech phases.
Based on VoiceDialogue's state management approach.
"""

from enum import Enum
from dataclasses import dataclass
from typing import Optional
import time
import uuid


class SpeechState(Enum):
    """Speech detection states"""
    SILENCE = "silence"
    SPEAKING = "speaking"
    TRAILING_SILENCE = "trailing_silence"


@dataclass
class VoiceTask:
    """
    Voice task representation from VoiceDialogue.
    Represents a speech segment with metadata.
    """
    id: str
    session_id: str
    answer_id: str
    user_voice: Optional[object] = None  # numpy array
    send_time: float = 0.0
    is_over_audio_frames_threshold: bool = False
    
    @classmethod
    def create(cls, task_id: str, session_id: str, audio_data=None):
        """Create a new voice task"""
        return cls(
            id=task_id,
            session_id=session_id,
            answer_id=str(uuid.uuid4()),
            user_voice=audio_data,
            send_time=time.time()
        )


class SpeechStateMachine:
    """
    Manages speech state transitions and tracking.
    """
    
    def __init__(self):
        self.state = SpeechState.SILENCE
        self.task_id: Optional[str] = None
        self.session_id: str = str(uuid.uuid4())
        
        # Duration tracking (milliseconds)
        self.active_audio_frame_duration = 0.0
        self.user_silence_duration = 0.0
        self.silence_audio_frame_count = 0
        
        # Flags
        self.is_audio_sent_for_processing = False
        self.is_audio_frames_empty = True
        
    def reset(self):
        """Reset state machine to initial state"""
        self.state = SpeechState.SILENCE
        self.active_audio_frame_duration = 0.0
        self.user_silence_duration = 0.0
        self.silence_audio_frame_count = 0
        self.is_audio_sent_for_processing = False
        self.is_audio_frames_empty = True
        
    def create_task_id(self):
        """Create new task ID"""
        self.task_id = str(uuid.uuid4())
        return self.task_id
        
    def transition_to_speaking(self):
        """Transition to speaking state"""
        if self.state == SpeechState.SILENCE:
            self.state = SpeechState.SPEAKING
            self.create_task_id()
            self.user_silence_duration = 0.0
            return True
        elif self.state == SpeechState.TRAILING_SILENCE:
            self.state = SpeechState.SPEAKING
            self.user_silence_duration = 0.0
            return False  # Same speech segment
        return False
        
    def transition_to_trailing_silence(self):
        """Transition to trailing silence state"""
        if self.state == SpeechState.SPEAKING:
            self.state = SpeechState.TRAILING_SILENCE
            return True
        return False
        
    def transition_to_silence(self):
        """Transition to silence state"""
        if self.state == SpeechState.TRAILING_SILENCE:
            self.state = SpeechState.SILENCE
            # Reset for next speech segment
            self.active_audio_frame_duration = 0.0
            self.is_audio_sent_for_processing = False
            self.is_audio_frames_empty = True
            return True
        return False
        
    def update_active_duration(self, duration_ms: float):
        """Update active speech duration"""
        self.active_audio_frame_duration += duration_ms
        
    def update_silence_duration(self, duration_ms: float):
        """Update silence duration"""
        self.user_silence_duration += duration_ms
        
    def is_user_in_silence(self, threshold_ms: float) -> bool:
        """Check if user has been silent long enough"""
        return self.user_silence_duration >= threshold_ms