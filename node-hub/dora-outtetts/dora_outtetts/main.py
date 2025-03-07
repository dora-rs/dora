from __future__ import annotations

import argparse  # Add argparse import
import os
import pathlib

import outetts
import pyarrow as pa
import torch
from dora import Node

PATH_SPEAKER = os.getenv("PATH_SPEAKER", "speaker.json")

device = "mps" if torch.backends.mps.is_available() else "cpu"
device = "cuda:0" if torch.cuda.is_available() else device
torch_dtype = torch.float16 if torch.cuda.is_available() else torch.float32


def load_interface():
    if os.getenv("INTERFACE", "HF") == "HF":
        model_config = outetts.HFModelConfig_v1(
            model_path="OuteAI/OuteTTS-0.2-500M",
            language="en",
            device=device,
        )

        interface = outetts.InterfaceHF(model_version="0.2", cfg=model_config)
    else:
        model_config = outetts.GGUFModelConfig_v1(
            model_path=os.getenv(
                "GGUF_MODEL_PATH",
                "~/.cache/huggingface/hub/models--OuteAI--OuteTTS-0.2-500M-GGUF/snapshots/e6d78720d2a8edce2bc8f5c5c2d0332e57091930/OuteTTS-0.2-500M-Q4_0.gguf",
            ),
            language="en",  # Supported languages in v0.2: en, zh, ja, ko
            n_gpu_layers=0,
        )

        interface = outetts.InterfaceGGUF(model_version="0.2", cfg=model_config)

    return interface


def create_speaker(interface, path) -> None:
    speaker = interface.create_speaker(
        audio_path=path,
        # If transcript is not provided, it will be automatically transcribed using Whisper
        transcript=None,  # Set to None to use Whisper for transcription
        whisper_model="turbo",  # Optional: specify Whisper model (default: "turbo")
        whisper_device=None,  # Optional: specify device for Whisper (default: None)
    )
    interface.save_speaker(speaker, "speaker.json")



def main(arg_list: list[str] | None = None) -> None:
    # Parse cli args
    parser = argparse.ArgumentParser(description="Dora Outetts Node")
    parser.add_argument("--create-speaker", type=str, help="Path to audio file")
    parser.add_argument("--test", action="store_true", help="Run tests")
    args = parser.parse_args(arg_list)
    if args.test:
        import pytest

        path = pathlib.Path(__file__).parent.resolve()
        pytest.main(["-x", path / "tests"])

        return

    interface = load_interface()

    if args.create_speaker:
        create_speaker(interface, args.create_speaker)
        return

    if os.path.exists(PATH_SPEAKER):
        # speaker = interface.load_speaker(PATH_SPEAKER)
        speaker = interface.load_default_speaker(name="male_1")
    else:
        # Load default speaker
        speaker = interface.load_default_speaker(name="male_1")

    node = Node()

    for event in node:
        if event["type"] == "INPUT":
            if event["id"] == "TICK":
                pass

            elif event["id"] == "text":
                # Warning: Make sure to add my_output_id and my_input_id within the dataflow.
                text = event["value"][0].as_py()
                output = interface.generate(
                    text=text,
                    temperature=0.1,
                    repetition_penalty=1.1,
                    speaker=speaker,  # Optional: speaker profile
                )
                node.send_output(
                    "audio",
                    pa.array(output.audio.cpu().numpy().ravel()),
                    {"language": "en", "sample_rate": output.sr},
                )


if __name__ == "__main__":
    main()
