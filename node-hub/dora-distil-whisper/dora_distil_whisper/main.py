import torch
from transformers import AutoModelForSpeechSeq2Seq, AutoProcessor, pipeline
from dora import Node
import pyarrow as pa
import os
from pathlib import Path

DEFAULT_PATH = "openai/whisper-large-v3-turbo"
TARGET_LANGUAGE = os.getenv("TARGET_LANGUAGE", "chinese")
TRANSLATE = bool(os.getenv("TRANSLATE", "False") in ["True", "true"])


MODEL_NAME_OR_PATH = os.getenv("MODEL_NAME_OR_PATH", DEFAULT_PATH)

if bool(os.getenv("USE_MODELSCOPE_HUB") in ["True", "true"]):
    from modelscope import snapshot_download

    if not Path(MODEL_NAME_OR_PATH).exists():
        MODEL_NAME_OR_PATH = snapshot_download(MODEL_NAME_OR_PATH)

device = "cuda:0" if torch.cuda.is_available() else "cpu"
torch_dtype = torch.float16 if torch.cuda.is_available() else torch.float32


model = AutoModelForSpeechSeq2Seq.from_pretrained(
    MODEL_NAME_OR_PATH,
    torch_dtype=torch_dtype,
    low_cpu_mem_usage=True,
    use_safetensors=True,
)
model.to(device)

processor = AutoProcessor.from_pretrained(MODEL_NAME_OR_PATH)
pipe = pipeline(
    "automatic-speech-recognition",
    model=model,
    tokenizer=processor.tokenizer,
    feature_extractor=processor.feature_extractor,
    max_new_tokens=400,
    torch_dtype=torch_dtype,
    device=device,
)

BAD_SENTENCES = [
    "字幕",
    "字幕志愿",
    "中文字幕",
    "我",
    "你",
    "THANK YOU",
    " Thank you.",
    " www.microsoft.com",
    " The",
    " BANG",
    " Silence.",
    " Sous-titrage Société Radio-Canada",
    " Sous",
    " Sous-",
]


def cut_repetition(text, min_repeat_length=4, max_repeat_length=50):
    # Check if the text is primarily Chinese (you may need to adjust this threshold)
    if sum(1 for char in text if "\u4e00" <= char <= "\u9fff") / len(text) > 0.5:
        # Chinese text processing
        for repeat_length in range(
            min_repeat_length, min(max_repeat_length, len(text) // 2)
        ):
            for i in range(len(text) - repeat_length * 2 + 1):
                chunk1 = text[i : i + repeat_length]
                chunk2 = text[i + repeat_length : i + repeat_length * 2]

                if chunk1 == chunk2:
                    return text[: i + repeat_length]
    else:
        # Non-Chinese (space-separated) text processing
        words = text.split()
        for repeat_length in range(
            min_repeat_length, min(max_repeat_length, len(words) // 2)
        ):
            for i in range(len(words) - repeat_length * 2 + 1):
                chunk1 = " ".join(words[i : i + repeat_length])
                chunk2 = " ".join(words[i + repeat_length : i + repeat_length * 2])

                if chunk1 == chunk2:
                    return " ".join(words[: i + repeat_length])

    return text


def main():
    node = Node()
    for event in node:
        if event["type"] == "INPUT":
            audio = event["value"].to_numpy()
            confg = (
                {"language": TARGET_LANGUAGE, "task": "translate"}
                if TRANSLATE
                else {
                    "language": TARGET_LANGUAGE,
                }
            )
            result = pipe(
                audio,
                generate_kwargs=confg,
            )
            if result["text"] in BAD_SENTENCES:
                continue
            text = cut_repetition(result["text"])
            node.send_output("text", pa.array([text]), {"language": TARGET_LANGUAGE})
