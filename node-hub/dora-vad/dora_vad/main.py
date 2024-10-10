from dora import Node
import pyarrow as pa
import numpy as np
import os
from silero_vad import load_silero_vad, get_speech_timestamps
import torch

model = load_silero_vad()
MIN_SILENCE_DURATION_MS = int(os.getenv("MIN_SILENCE_DURATION_MS", "200"))
MIN_SPEECH_DURATION_MS = int(os.getenv("MIN_SPEECH_DURATION_MS", "1000"))

MIN_AUDIO_SAMPLING_DURAION_S = int(os.getenv("MAX_AUDIO_DURATION_S", "20"))
MAX_AUDIO_DURAION_S = int(os.getenv("MAX_AUDIO_DURATION_S", "75"))


def main():
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
            speech_timestamps = get_speech_timestamps(
                torch.from_numpy(audio),
                model,
                threshold=0.2,
                min_speech_duration_ms=MIN_SPEECH_DURATION_MS,
                min_silence_duration_ms=MIN_SILENCE_DURATION_MS,
            )

            # Check ig there is timestamp
            if (
                len(speech_timestamps) > 0
                and len(last_audios) > MIN_AUDIO_SAMPLING_DURAION_S
            ):

                # Check if the audio is not cut at the end. And only return if there is a long time spent
                if speech_timestamps[-1]["end"] == len(audio):
                    continue
                else:
                    audio = audio[0 : speech_timestamps[-1]["end"]]
                    node.send_output("audio", pa.array(audio))
                    last_audios = [audio[speech_timestamps[-1]["end"] :]]

            # If there is no sound for too long return the audio
            elif len(last_audios) > 75:
                node.send_output("audio", pa.array(audio))
                last_audios = []
