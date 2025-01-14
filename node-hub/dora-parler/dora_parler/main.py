import os
import time
from pathlib import Path
from threading import Thread

import numpy as np
import pyaudio
import torch
from dora import Node
from parler_tts import ParlerTTSForConditionalGeneration, ParlerTTSStreamer
from transformers import (
    AutoFeatureExtractor,
    AutoTokenizer,
    StoppingCriteria,
    StoppingCriteriaList,
    set_seed,
)

device = "cuda:0"  # if torch.cuda.is_available() else "mps" if torch.backends.mps.is_available() else "cpu"
torch_dtype = torch.float16 if device != "cpu" else torch.float32

DEFAULT_PATH = "ylacombe/parler-tts-mini-jenny-30H"


MODEL_NAME_OR_PATH = os.getenv("MODEL_NAME_OR_PATH", DEFAULT_PATH)

if bool(os.getenv("USE_MODELSCOPE_HUB") in ["True", "true"]):
    from modelscope import snapshot_download

    if not Path(MODEL_NAME_OR_PATH).exists():
        MODEL_NAME_OR_PATH = snapshot_download(MODEL_NAME_OR_PATH)

model = ParlerTTSForConditionalGeneration.from_pretrained(
    MODEL_NAME_OR_PATH, torch_dtype=torch_dtype, low_cpu_mem_usage=True,
).to(device)
model.generation_config.cache_implementation = "static"
model.forward = torch.compile(model.forward, mode="default")

tokenizer = AutoTokenizer.from_pretrained(MODEL_NAME_OR_PATH)
feature_extractor = AutoFeatureExtractor.from_pretrained(MODEL_NAME_OR_PATH)

SAMPLE_RATE = feature_extractor.sampling_rate
SEED = 42

default_text = "Hello, my name is Reachy the best robot in the world !"
default_description = (
    "Jenny delivers her words quite expressively, in a very confined sounding environment with clear audio quality.",
)


p = pyaudio.PyAudio()


sampling_rate = model.audio_encoder.config.sampling_rate
frame_rate = model.audio_encoder.config.frame_rate

stream = p.open(format=pyaudio.paInt16, channels=1, rate=sampling_rate, output=True)


def play_audio(audio_array):
    if np.issubdtype(audio_array.dtype, np.floating):
        max_val = np.max(np.abs(audio_array))
        audio_array = (audio_array / max_val) * 32767
        audio_array = audio_array.astype(np.int16)

    stream.write(audio_array.tobytes())


class InterruptStoppingCriteria(StoppingCriteria):
    def __init__(self):
        super().__init__()
        self.stop_signal = False

    def __call__(
        self, input_ids: torch.LongTensor, scores: torch.FloatTensor, **kwargs,
    ) -> bool:
        return self.stop_signal

    def stop(self):
        self.stop_signal = True


def generate_base(
    node,
    text=default_text,
    description=default_description,
    play_steps_in_s=0.5,
):
    prev_time = time.time()
    play_steps = int(frame_rate * play_steps_in_s)
    inputs = tokenizer(description, return_tensors="pt").to(device)
    prompt = tokenizer(text, return_tensors="pt").to(device)
    streamer = ParlerTTSStreamer(model, device=device, play_steps=play_steps)

    stopping_criteria = InterruptStoppingCriteria()

    generation_kwargs = dict(
        input_ids=inputs.input_ids,
        prompt_input_ids=prompt.input_ids,
        streamer=streamer,
        do_sample=True,
        temperature=1.0,
        min_new_tokens=10,
        stopping_criteria=StoppingCriteriaList([stopping_criteria]),
    )
    set_seed(SEED)
    thread = Thread(target=model.generate, kwargs=generation_kwargs)
    thread.start()

    for new_audio in streamer:
        current_time = time.time()

        print(f"Time between iterations: {round(current_time - prev_time, 2)} seconds")
        prev_time = current_time
        play_audio(new_audio)

        if node is None:
            continue

        event = node.next(timeout=0.01)

        if event["type"] == "ERROR":
            pass
        elif event["type"] == "INPUT":
            if event["id"] == "stop":
                stopping_criteria.stop()
                break
            if event["id"] == "text":
                stopping_criteria.stop()

                text = event["value"][0].as_py()
                generate_base(node, text, default_description, 0.5)


def main():
    generate_base(None, "Ready !", default_description, 0.5)
    node = Node()
    while True:
        event = node.next()
        if event is None:
            break
        if event["type"] == "INPUT" and event["id"] == "text":
            text = event["value"][0].as_py()
            generate_base(node, text, default_description, 0.5)

    stream.stop_stream()
    stream.close()
    p.terminate()


if __name__ == "__main__":
    main()
