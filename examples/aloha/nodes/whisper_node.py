import pyarrow as pa
import whisper
from pynput import keyboard
from pynput.keyboard import Key, Events
from dora import Node

import torch
import numpy as np
import pyarrow as pa
import sounddevice as sd
import gc  # garbage collect library

model = whisper.load_model("base")

SAMPLE_RATE = 16000

node = Node()


def get_text(duration) -> str:

    ## Microphone
    audio_data = sd.rec(
        int(SAMPLE_RATE * duration),
        samplerate=SAMPLE_RATE,
        channels=1,
        dtype=np.int16,
        blocking=True,
    )

    audio = audio_data.ravel().astype(np.float32) / 32768.0

    ## Speech to text
    audio = whisper.pad_or_trim(audio)
    return model.transcribe(audio, language="en")


## Check for keyboard event
with keyboard.Events() as events:
    for dora_event in node:
        if dora_event["type"] == "INPUT":
            event = events.get(0.1)
            if (
                event is not None
                and (event.key == Key.alt_r or event.key == Key.ctrl_r)
                and isinstance(event, Events.Press)
            ):
                if event.key == Key.alt_r:
                    result = get_text(5)
                    node.send_output(
                        "text_llm", pa.array([result["text"]]), dora_event["metadata"]
                    )
                elif event.key == Key.ctrl_r:
                    result = get_text(3)
                    node.send_output(
                        "text_policy",
                        pa.array([result["text"]]),
                        dora_event["metadata"],
                    )
