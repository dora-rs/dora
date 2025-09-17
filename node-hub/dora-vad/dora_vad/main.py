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
    tmp_audio = []
    should_stop = 0
    last_speech_timestamps = []
    while True:
        event = node.next()
        if event is None:
            break
        if event["type"] == "INPUT" and event["id"] == "audio":
            audio = event["value"].to_numpy()
            sr = event["metadata"].get("sample_rate", 16000)

            tmp_audio += [audio]
            if (len(tmp_audio) * len(audio) / (sr)) < 0.1:
                continue
            audio = np.concatenate(tmp_audio)
            last_audios += [audio]
            tmp_audio = []

            if (len(audio) / sr) < 0.1:
                continue
            # If the block of audio are less than 100ms skip some of them
            if ((10 * sr) // len(audio)) < len(last_audios) and should_stop == 0:
                last_audios = last_audios[-(10 * sr) // len(audio) :]

            audio = np.concatenate(last_audios)
            speech_timestamps = get_speech_timestamps(
                torch.from_numpy(audio),
                model,
                threshold=THRESHOLD,
                min_speech_duration_ms=MIN_SPEECH_DURATION_MS,
                min_silence_duration_ms=MIN_SILENCE_DURATION_MS,
                sampling_rate=sr,
                speech_pad_ms=200,
            )
            if len(speech_timestamps) == 0 and should_stop > 0:
                should_stop += 1
            if len(speech_timestamps) == 0:
                if should_stop < 5:
                    # If there is no speech, return the audio
                    continue
                speech_timestamps = last_speech_timestamps
            if speech_timestamps[-1]["end"] == len(audio):
                # If the speech is at the end of the audio, wait for more audio
                should_stop += 1
                last_speech_timestamps = speech_timestamps
                continue

            should_stop = 0
            # Check ig there is timestamp
            node.send_output(
                "audio",
                pa.array(
                    audio  # [speech_timestamps[0]["start"] : speech_timestamps[-1]["end"]]
                ),
                metadata={"sample_rate": sr},
            )
            last_audios = []
