"""TODO: Add docstring."""

import os
import re
import sys
import time
from pathlib import Path

import numpy as np
import pyarrow as pa
import torch
from dora import Node

DEFAULT_PATH = "openai/whisper-large-v3-turbo"
TARGET_LANGUAGE = os.getenv("TARGET_LANGUAGE", "english")
TRANSLATE = bool(os.getenv("TRANSLATE", "False") in ["True", "true"])


def remove_text_noise(text: str, text_noise="") -> str:
    """Remove noise from text.

    Args:
        text (str): Original text
        text_noise (str): text to remove from the original text

    Returns:
        str: Cleaned text

    """
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
        return re.sub(r"[^\w\s]", "", s).lower()

    # Normalize both text and text_noise
    normalized_text = normalize(text)
    normalized_noise = normalize(text_noise)

    # Split into words
    text_words = normalized_text.split()
    noise_words = normalized_noise.split()

    # Only remove parts of text_noise that are found in text
    cleaned_words = text_words[:]
    for noise_word in noise_words:
        if noise_word in cleaned_words:
            cleaned_words.remove(noise_word)

    # Reconstruct the cleaned text
    return " ".join(cleaned_words)


def load_model():
    """TODO: Add docstring."""
    from transformers import AutoModelForSpeechSeq2Seq, AutoProcessor, pipeline

    model_name_or_path = os.getenv("MODEL_NAME_OR_PATH", DEFAULT_PATH)

    if bool(os.getenv("USE_MODELSCOPE_HUB") in ["True", "true"]):
        from modelscope import snapshot_download

        if not Path(model_name_or_path).exists():
            model_name_or_path = snapshot_download(model_name_or_path)

    device = "cuda:0" if torch.cuda.is_available() else "cpu"
    torch_dtype = torch.float16 if torch.cuda.is_available() else torch.float32

    model = AutoModelForSpeechSeq2Seq.from_pretrained(
        model_name_or_path,
        torch_dtype=torch_dtype,
        low_cpu_mem_usage=True,
        use_safetensors=True,
    )
    model.to(device)

    processor = AutoProcessor.from_pretrained(model_name_or_path)
    return pipeline(
        "automatic-speech-recognition",
        model=model,
        tokenizer=processor.tokenizer,
        feature_extractor=processor.feature_extractor,
        max_new_tokens=400,
        torch_dtype=torch_dtype,
        device=device,
    )


BAD_SENTENCES = [
    "",
    " so",
    " So.",
    " So, let's go.",
    " so so",
    " What?",
    " We'll see you next time.",
    " I'll see you next time.",
    " We're going to come back.",
    " let's move on.",
    " Here we go.",
    " my",
    " All right. Thank you.",
    " That's what we're doing.",
    " That's what I wanted to do.",
    " I'll be back.",
    " Hold this. Hold this.",
    " Hold this one. Hold this one.",
    " And we'll see you next time.",
    " strength.",
    " Length.",
    " Let's go.",
    " Let's do it.",
    "You",
    "You ",
    " You",
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
    text_noise = ""
    # For macos use mlx:
    if sys.platform != "darwin":
        pipe = load_model()
    else:
        import mlx_whisper

        result = mlx_whisper.transcribe(
            np.array([]),
            path_or_hf_repo="mlx-community/whisper-large-v3-turbo",
            append_punctuations=".",
            language=TARGET_LANGUAGE,
        )
        result = mlx_whisper.transcribe(
            np.array([]),
            path_or_hf_repo="mlx-community/whisper-large-v3-turbo",
            append_punctuations=".",
            language=TARGET_LANGUAGE,
        )

    node = Node()
    noise_timestamp = time.time()
    cache_audio = None
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
                audio_input = event["value"].to_numpy()
                if cache_audio is not None:
                    audio = np.concatenate([cache_audio, audio_input])
                else:
                    audio = audio_input

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
                        language=TARGET_LANGUAGE,
                    )

                else:
                    result = pipe(
                        audio,
                        generate_kwargs=confg,
                    )
                if result["text"] in BAD_SENTENCES:
                    print("Discarded text: ", result["text"])
                    # cache_audio = None
                    continue
                text = cut_repetition(result["text"])

                # Remove noise filter after some time
                if time.time() - noise_timestamp > (len(text_noise.split()) / 2):  # WPS
                    text_noise = ""

                ## Remove text noise independently of casing
                text = remove_text_noise(text, text_noise)

                if text.strip() == "" or text.strip() == ".":
                    continue

                if (
                    text.endswith(".")
                    or text.endswith("!")
                    or text.endswith("?")
                    or text.endswith('."')
                    or text.endswith('!"')
                    or text.endswith('?"')
                ) and not text.endswith("..."):
                    node.send_output(
                        "text",
                        pa.array([text]),
                    )
                    node.send_output(
                        "stop",
                        pa.array([text]),
                    )
                    cache_audio = None
                    audio = None
                    print("Text:", text)
                elif text.endswith("..."):
                    print(
                        "Keeping audio in cache for next text output with punctuation"
                    )
                    print("Discarded text", text)
                    cache_audio = audio
