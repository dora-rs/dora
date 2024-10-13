import sounddevice as sd
import numpy as np
import pyarrow as pa
import time as tm
import os

from dora import Node

MAX_DURATION = float(os.getenv("MAX_DURATION", "0.1"))
SAMPLE_RATE = int(os.getenv("SAMPLE_RATE", "16000"))


def main():
    # Initialize buffer and recording flag
    buffer = []
    start_recording_time = tm.time()
    node = Node()

    # pylint: disable=unused-argument
    def callback(indata, frames, time, status):
        nonlocal buffer, node, start_recording_time

        if tm.time() - start_recording_time > MAX_DURATION:
            audio_data = np.array(buffer).ravel().astype(np.float32) / 32768.0
            node.send_output("audio", pa.array(audio_data))
            buffer = []
            start_recording_time = tm.time()
        else:
            buffer.extend(indata[:, 0])

    # Start recording
    with sd.InputStream(
        callback=callback, dtype=np.int16, channels=1, samplerate=SAMPLE_RATE
    ):
        while True:
            sd.sleep(int(100 * 1000))
