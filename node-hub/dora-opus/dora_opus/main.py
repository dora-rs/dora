import os
from pathlib import Path
from dora import Node
import pyarrow as pa
import numpy as np
from transformers import AutoTokenizer, AutoModelForSeq2SeqLM


from_code = os.getenv("SOURCE_LANGUAGE", "zh")
to_code = os.getenv("TARGET_LANGUAGE", "en")
DEFAULT_PATH = f"Helsinki-NLP/opus-mt-{from_code}-{to_code}"


MODEL_NAME_OR_PATH = os.getenv("MODEL_NAME_OR_PATH", DEFAULT_PATH)

if bool(os.getenv("USE_MODELSCOPE_HUB") in ["True", "true"]):
    from modelscope import snapshot_download

    if not Path(MODEL_NAME_OR_PATH).exists():
        MODEL_NAME_OR_PATH = snapshot_download(MODEL_NAME_OR_PATH)

tokenizer = AutoTokenizer.from_pretrained(MODEL_NAME_OR_PATH)

model = AutoModelForSeq2SeqLM.from_pretrained(MODEL_NAME_OR_PATH)


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
    while True:
        event = node.next()
        if event is None:
            break
        if event["type"] == "INPUT" and event["id"] == "text":
            text = [str(event["value"][0].as_py())]
            translated = (
                model.generate(**tokenizer(text, return_tensors="pt", padding=True))
                .to("cpu")
                .detach()
                .numpy()
                .ravel()
            )

            array = np.array(tokenizer.decode(translated, skip_special_tokens=True))
            array = np.array(array).ravel()
            array = [cut_repetition(array[0])]
            node.send_output(
                "text",
                pa.array(array),
                {"language": to_code},
            )
