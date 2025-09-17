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
                sampling_rate=sr,
            )
            if len(speech_timestamps) == 0:
                # If there is no speech, return the audio
                continue
            # Send a timestamp_start
            node.send_output(
                "timestamp_start",
                pa.array([speech_timestamps[0]["start"]]),
                metadata={"sample_rate": sr},
            )

            for i, ts in enumerate(reversed(speech_timestamps)):
                # Convert timestamps to milliseconds
                speech_timestamps[i]["start"] = int(ts["start"] / sr * 1000)
                speech_timestamps[i]["end"] = int(ts["end"] / sr * 1000)
                if (ts["end"] - ts["start"]) > MIN_SPEECH_DURATION_MS:
                    arg_max = len(speech_timestamps) - i
                    break
            # Check ig there is timestamp
            if (
                len(speech_timestamps) > 0
                and len(
                    audio[speech_timestamps[0]["start"] : speech_timestamps[-1]["end"]]
                )
                > MIN_AUDIO_SAMPLING_DURATION_MS * sr / 1000
                and (
                    (len(audio) - speech_timestamps[arg_max]["end"])
                    > MIN_SILENCE_DURATION_MS / 1000 * sr
                )
            ):
                # Check if the audio is not cut at the end. And only return if there is a long time spent
                if speech_timestamps[-1]["end"] == len(audio):
                    node.send_output(
                        "timestamp_start",
                        pa.array([speech_timestamps[-1]["start"]]),
                        metadata={"sample_rate": sr},
                    )
                audio = audio[: speech_timestamps[-1]["end"]]
                node.send_output("audio", pa.array(audio), metadata={"sample_rate": sr})
                last_audios = [audio[speech_timestamps[-1]["end"] :]]

            # If there is no sound for too long return the audio
            elif len(last_audios) > MAX_AUDIO_DURATION_S:
                node.send_output("audio", pa.array(audio), metadata={"sample_rate": sr})
                last_audios = []
