from doraflow import Dataflow

# This example is a translation of the YAML file at
# examples/translation/dataflow_zh_en_terminal.yml

with Dataflow(name="translation-dataflow") as df:
    microphone = df.add_node(
        id="dora-microphone",
        build="pip install -e ../../node-hub/dora-microphone",
        path="dora-microphone",
    )

    vad = df.add_node(
        id="dora-vad",
        build="pip install -e ../../node-hub/dora-vad",
        path="dora-vad",
    )

    whisper = df.add_node(
        id="dora-distil-whisper",
        build="pip install -e ../../node-hub/dora-distil-whisper",
        path="dora-distil-whisper",
        env={"TARGET_LANGUAGE": "chinese", "TRANSLATE": "false"},
    )

    opus = df.add_node(
        id="dora-opus",
        build="pip install -e ../../node-hub/dora-opus",
        path="dora-opus",
        env={"SOURCE_LANGUAGE": "zh", "TARGET_LANGUAGE": "en"},
    )

    pretty_print = df.add_node(
        id="pretty-print",
        path="dynamic",
    )

    # Connect the nodes
    df.add_edge(microphone, "audio", vad, "audio")
    df.add_edge(vad, "audio", whisper, "input")
    df.add_edge(whisper, "text", opus, "text")
    df.add_edge(opus, "text", pretty_print, "translated_text")
    df.add_edge(whisper, "text", pretty_print, "original_text")

    # Generate the YAML file
    df.to_yaml("translation_dataflow.yml")

    print("Generated translation_dataflow.yml")

    # Visualize the dataflow
    df.visualize()

    # You can now run this dataflow with:
    # dora up
    # dora start translation_dataflow.yml
