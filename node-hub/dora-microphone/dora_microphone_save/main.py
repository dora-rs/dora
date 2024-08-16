import sounddevice as sd
import numpy as np
import pyarrow as pa
import time as tm
from enum import Enum

from dora import Node


class RecordingState(Enum):
    """Enum for recording states."""

    PENDING = 0
    RUNNING = 1
    SILENCE = 2


def detect_speech(audio_data, threshold):
    """Check if the amplitude of the audio signal exceeds the threshold."""
    return np.any(np.abs(audio_data) > threshold)


def main():
    # Parameters
    threshold = 500  # Threshold for detecting speech (adjust this as needed)
    SAMPLE_RATE = 16000
    silence_duration = 4  # Duration of silence before stopping the recording

    # Initialize buffer and recording flag
    buffer = []
    state = RecordingState.PENDING
    silence_start_time = tm.time()
    node = Node()

    def callback(indata, frames, time, status):
        nonlocal buffer, state, silence_start_time, node

        is_speaking = detect_speech(indata[:, 0], threshold)
        if is_speaking:
            if state == RecordingState.PENDING:
                buffer = []
                state = RecordingState.RUNNING
            buffer.extend(indata[:, 0])
        elif not is_speaking and state == RecordingState.RUNNING:
            silence_start_time = tm.time()  # Reset silence timer
            buffer.extend(indata[:, 0])
            state = RecordingState.SILENCE
        elif not is_speaking and state == RecordingState.SILENCE:
            if tm.time() - silence_start_time > silence_duration:
                audio_data = np.array(buffer).ravel().astype(np.float32) / 32768.0
                node.send_output("audio", pa.array(audio_data))
                state = RecordingState.PENDING
            else:
                buffer.extend(indata[:, 0])

    # Start recording
    with sd.InputStream(
        callback=callback, dtype=np.int16, channels=1, samplerate=SAMPLE_RATE
    ):
        while True:
            sd.sleep(int(100 * 1000))
