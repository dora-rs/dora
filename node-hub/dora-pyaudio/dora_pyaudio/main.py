import os

import numpy as np
import pyarrow as pa
import pyaudio
from dora import Node

SAMPLE_RATE = int(os.getenv("SAMPLE_RATE", "16000"))


p = pyaudio.PyAudio()


def play_audio(
    audio_array: pa.array,
    sample_rate: int,
    stream: pyaudio.Stream = None,
) -> pyaudio.Stream:
    """Play audio using pyaudio and replace stream if already exists"""
    if np.issubdtype(audio_array.dtype, np.floating):
        max_val = np.max(np.abs(audio_array))
        audio_array = (audio_array / max_val) * 32767
        audio_array = audio_array.astype(np.int16)
    if stream is None:
        stream = p.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=sample_rate,
            output=True,
        )
    stream.write(audio_array.tobytes())
    return stream


def main():
    """Main function for the node"""
    node = Node()
    stream = None
    for event in node:
        if event["type"] == "INPUT":
            if event["id"] == "audio":
                audio = event["value"].to_numpy()
                sr = event["metadata"].get("sample_rate", SAMPLE_RATE)
                stream = play_audio(audio, sr, stream)

    if stream is not None:
        stream.stop_stream()
        stream.close()
        p.terminate()


if __name__ == "__main__":
    main()
