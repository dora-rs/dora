from dora import Node
import pyarrow as pa
import pyaudio
import os
import numpy as np

SAMPLE_RATE = os.getenv("SAMPLE_RATE", 16000)


p = pyaudio.PyAudio()


def play_audio(audio_array: pa.array, sample_rate: int, stream):

    if np.issubdtype(audio_array.dtype, np.floating):
        max_val = np.max(np.abs(audio_array))
        audio_array = (audio_array / max_val) * 32767
        audio_array = audio_array.astype(np.int16)
    if stream is None:
        stream = p.open(
            format=pyaudio.paInt16, channels=1, rate=sample_rate, output=True
        )
    stream.write(audio_array.tobytes())
    return stream


def main():
    node = Node()
    stream = None
    for event in node:
        if event["type"] == "INPUT":
            if event["id"] == "audio":
                audio = event["value"].to_numpy()
                sr = event["metadata"].get("sample_rate", SAMPLE_RATE)
                stream = play_audio(audio, sr, stream)

    stream.stop_stream()
    stream.close()
    p.terminate()


if __name__ == "__main__":
    main()
