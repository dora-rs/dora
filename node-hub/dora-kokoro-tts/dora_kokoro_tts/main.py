"""TODO: Add docstring."""
import os
import re

import pyarrow as pa
from dora import Node
from kokoro import KPipeline

LANGUAGE = os.getenv("LANGUAGE", "en")

def main():
    """TODO: Add docstring."""
    if LANGUAGE in ["en", "english"]:
        pipeline = KPipeline(lang_code="a")
    elif LANGUAGE in ["zh","ch","chinese"]:
        pipeline = KPipeline(lang_code="z")
    else:
        print("warning: Defaulting to english speaker as language not found")
        pipeline = KPipeline(lang_code="a")

    node = Node()

    for event in node:
        if event["type"] == "INPUT":
            if event["id"] == "text":
                text = event["value"][0].as_py()
                if re.findall(r"[\u4e00-\u9fff]+", text):
                    pipeline = KPipeline(lang_code="z")
                elif pipeline.lang_code != "a":
                    pipeline = KPipeline(lang_code="a")  # <= make sure lang_code matches voice

                generator = pipeline(
                    text,
                    voice="af_heart",  # <= change voice here
                    speed=1.2,
                    split_pattern=r"\n+",
                )
                for _, (_, _, audio) in enumerate(generator):
                    audio = audio.numpy()
                    node.send_output("audio", pa.array(audio), {"sample_rate": 24000})


if __name__ == "__main__":
    main()
