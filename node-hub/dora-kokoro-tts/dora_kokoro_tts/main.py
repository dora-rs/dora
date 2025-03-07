import pyarrow as pa
from dora import Node
from kokoro import KPipeline

pipeline = KPipeline(lang_code="a")  # <= make sure lang_code matches voice


def main() -> None:
    node = Node()

    for event in node:
        if event["type"] == "INPUT" and event["id"] == "text":
            text = event["value"][0].as_py()
            generator = pipeline(
                text,
                voice="af_heart",  # <= change voice here
                speed=1,
                split_pattern=r"\n+",
            )
            for _i, (_gs, _ps, audio) in enumerate(generator):
                audio = audio.numpy()
                node.send_output("audio", pa.array(audio), {"sample_rate": 24000})


if __name__ == "__main__":
    main()
