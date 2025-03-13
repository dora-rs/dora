"""TODO: Add docstring."""

import os
import re
import sys
import time
from pathlib import Path

import pyarrow as pa
import torch
from dora import Node

DEFAULT_PATH = "openai/whisper-large-v3-turbo"
TARGET_LANGUAGE = os.getenv("TARGET_LANGUAGE", "english")
TRANSLATE = bool(os.getenv("TRANSLATE", "False") in ["True", "true"])


def remove_text_noise(text, text_noise):
    # Handle the case where text_noise is empty
    if not text_noise.strip():
        return (
            text  # Return the original text if text_noise is empty or just whitespace
        )

    # Helper function to normalize text (remove punctuation, make lowercase, and handle hyphens)
    def normalize(s):
        # Replace hyphens with spaces to treat "Notre-Dame" and "notre dame" as equivalent
        s = re.sub(r"-", " ", s)
        # Remove other punctuation and convert to lowercase
        s = re.sub(r"[^\w\s]", "", s).lower()
        return s

    # Normalize both text and text_noise
    normalized_text = normalize(text)
    normalized_noise = normalize(text_noise)

    # Split into words
    text_words = normalized_text.split()
    noise_words = normalized_noise.split()

    # Function to find and remove noise sequence flexibly
    def remove_flexible(text_list, noise_list):
        i = 0
        while i <= len(text_list) - len(noise_list):
            match = True
            extra_words = 0
            for j, noise_word in enumerate(noise_list):
                if i + j + extra_words >= len(text_list):
                    match = False
                    break
                # Allow skipping extra words in text_list
                while (
                    i + j + extra_words < len(text_list)
                    and text_list[i + j + extra_words] != noise_word
                ):
                    extra_words += 1
                    if i + j + extra_words >= len(text_list):
                        match = False
                        break
                if not match:
                    break
            if match:
                # Remove matched part
                del text_list[i : i + len(noise_list) + extra_words]
                i = max(0, i - len(noise_list))  # Adjust index after removal
            else:
                i += 1
        return text_list

    # Only remove parts of text_noise that are found in text
    cleaned_words = text_words[:]
    for noise_word in noise_words:
        if noise_word in cleaned_words:
            cleaned_words.remove(noise_word)

    # Reconstruct the cleaned text
    cleaned_text = " ".join(cleaned_words)
    return cleaned_text


def load_model():
    """TODO: Add docstring."""
    from transformers import AutoModelForSpeechSeq2Seq, AutoProcessor, pipeline

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
    return pipe


BAD_SENTENCES = [
    "",
    " so",
    " so so",
    "字幕",
    "字幕志愿",
    "中文字幕",
    "我",
    "你",
    " you",
    "!",
    "THANK YOU",
    " Thank you.",
    " www.microsoft.com",
    " The",
    " BANG",
    " Silence.",
    " Sous-titrage Société Radio-Canada",
    " Sous",
    " Sous-",
    " i'm going to go to the next one.",
]


def cut_repetition(text, min_repeat_length=4, max_repeat_length=50):
    """TODO: Add docstring."""
    if len(text) == 0:
        return text
    # Check if the text is primarily Chinese (you may need to adjust this threshold)
    if sum(1 for char in text if "\u4e00" <= char <= "\u9fff") / len(text) > 0.5:
        # Chinese text processing
        for repeat_length in range(
            min_repeat_length,
            min(max_repeat_length, len(text) // 2),
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
            min_repeat_length,
            min(max_repeat_length, len(words) // 2),
        ):
            for i in range(len(words) - repeat_length * 2 + 1):
                chunk1 = " ".join(words[i : i + repeat_length])
                chunk2 = " ".join(words[i + repeat_length : i + repeat_length * 2])

                if chunk1 == chunk2:
                    return " ".join(words[: i + repeat_length])

    return text


def main():
    """TODO: Add docstring."""
    node = Node()
    text_noise = ""
    noise_timestamp = time.time()
    # For macos use mlx:
    if sys.platform != "darwin":
        pipe = load_model()

    for event in node:
        if event["type"] == "INPUT":
            if "text_noise" in event["id"]:
                text_noise = event["value"][0].as_py()
                text_noise = (
                    text_noise.replace("(", "")
                    .replace(")", "")
                    .replace("[", "")
                    .replace("]", "")
                )
                noise_timestamp = time.time()
            else:
                audio = event["value"].to_numpy()
                confg = (
                    {"language": TARGET_LANGUAGE, "task": "translate"}
                    if TRANSLATE
                    else {
                        "language": TARGET_LANGUAGE,
                    }
                )
                if sys.platform == "darwin":
                    import mlx_whisper

                    result = mlx_whisper.transcribe(
                        audio,
                        path_or_hf_repo="mlx-community/whisper-large-v3-turbo",
                        append_punctuations=".",
                    )

                else:
                    result = pipe(
                        audio,
                        generate_kwargs=confg,
                    )
                if result["text"] in BAD_SENTENCES:
                    continue
                text = cut_repetition(result["text"])

                # Remove noise filter after some time
                if time.time() - noise_timestamp > (len(text_noise.split()) / 2):  # WPS
                    text_noise = ""

                ## Remove text noise independently of casing
                text = remove_text_noise(text, text_noise)

                if text.strip() == "" or text.strip() == ".":
                    continue
                node.send_output(
                    "text", pa.array([text]), {"language": TARGET_LANGUAGE}
                )
