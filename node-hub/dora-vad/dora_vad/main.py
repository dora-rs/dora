"""TODO: Add docstring."""

import os

import numpy as np
import pyarrow as pa
import torch
from dora import Node
from silero_vad import get_speech_timestamps, load_silero_vad

model = load_silero_vad()
MIN_SILENCE_DURATION_MS = int(os.getenv("MIN_SILENCE_DURATION_MS", "200"))
MIN_SPEECH_DURATION_MS = int(os.getenv("MIN_SPEECH_DURATION_MS", "300"))
THRESHOLD = float(os.getenv("THRESHOLD", "0.4"))
MAX_AUDIO_DURATION_S = float(os.getenv("MAX_AUDIO_DURATION_S", "75"))
MIN_AUDIO_SAMPLING_DURATION_MS = int(os.getenv("MIN_AUDIO_SAMPLING_DURATION_MS", "500"))


def main():
    """TODO: Add docstring."""
    node = Node()
    last_audios = []
    while True:
        event = node.next()
        if event is None:
            break
        if event["type"] == "INPUT" and event["id"] == "audio":
            audio = event["value"].to_numpy()
            last_audios += [audio]
            last_audios = last_audios[-100:]
            audio = np.concatenate(last_audios)
            sr = event["metadata"].get("sample_rate", 16000)
            speech_timestamps = get_speech_timestamps(
                torch.from_numpy(audio),
                model,
                threshold=THRESHOLD,
                min_speech_duration_ms=MIN_SPEECH_DURATION_MS,
                min_silence_duration_ms=MIN_SILENCE_DURATION_MS,
            )

            # Check ig there is timestamp
            if (
                len(speech_timestamps) > 0
                and len(audio) > MIN_AUDIO_SAMPLING_DURATION_MS * sr / 1000
            ):
                # Check if the audio is not cut at the end. And only return if there is a long time spent
                if speech_timestamps[-1]["end"] == len(audio):
                    node.send_output(
                        "timestamp_start",
                        pa.array([speech_timestamps[-1]["start"]]),
                    )
                    continue
                audio = audio[0 : speech_timestamps[-1]["end"]]
                node.send_output("audio", pa.array(audio), metadata={"sample_rate": sr})
                last_audios = [audio[speech_timestamps[-1]["end"] :]]

            # If there is no sound for too long return the audio
            elif len(last_audios) > MAX_AUDIO_DURATION_S:
                node.send_output("audio", pa.array(audio), metadata={"sample_rate": sr})
                last_audios = []
