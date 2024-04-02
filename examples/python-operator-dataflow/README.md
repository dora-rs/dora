# Python Dataflow Example

This examples shows how to create and connect dora operators and custom nodes in Python.

## Overview

The [`dataflow.yml`](./dataflow.yml) defines a simple dataflow graph with the following three nodes:

- a webcam node, that connects to your webcam and feed the dataflow with webcam frame as jpeg compressed bytearray.
- an object detection node, that apply Yolo v5 on the webcam image. The model is imported from Pytorch Hub. The output is the bouding box of each object detected, the confidence and the class. You can have more info here: https://pytorch.org/hub/ultralytics_yolov5/
- a window plotting node, that will retrieve the webcam image and the Yolov5 bounding box and join the two together.

## Getting started

```bash
cargo run --example python-operator-dataflow
```

## Installation

```bash
conda create -n example_env python=3.12
pip install -r requirements.txt
```

## Run the dataflow

- Start the object detection dataflow alone:

```bash
dora start dataflow.yml
```

- Start the llm dataflow:

```bash
dora start dataflow_llm.yml
```

Within the window you can ask question such as:

```bash
ask how are you
change bounding box plot to red
change confidence value to percentage
change object detection to only detect person
send 200 200 200 400 to topic line
record
```

```bash
wget https://raw.githubusercontent.com/dora-rs/dora/v0.3.2/examples/python-operator-dataflow/keyboard_op.py
wget https://raw.githubusercontent.com/dora-rs/dora/v0.3.2/examples/python-operator-dataflow/microphone_op.py
wget https://raw.githubusercontent.com/dora-rs/dora/v0.3.2/examples/python-operator-dataflow/whisper_op.py
wget https://raw.githubusercontent.com/dora-rs/dora/v0.3.2/examples/python-operator-dataflow/sentence_transformers_op.py
wget https://raw.githubusercontent.com/dora-rs/dora/v0.3.2/examples/python-operator-dataflow/llm_op.py
wget https://raw.githubusercontent.com/dora-rs/dora/v0.3.2/examples/python-operator-dataflow/file_saver_op.py
```

and adding the following to the dataflow configuration:

```yaml
nodes:
  - id: webcam
    operator:
      python: webcam.py
      inputs:
        tick: dora/timer/millis/50
      outputs:
        - image

  - id: object_detection
    operator:
      python: object_detection.py
      inputs:
        image: webcam/image
      outputs:
        - bbox

  - id: plot
    operator:
      python: plot.py
      inputs:
        image: webcam/image
        bbox: object_detection/bbox
        line: llm/line
        keyboard_buffer: keyboard/buffer
        user_message: keyboard/submitted
        assistant_message: llm/assistant_message

  ## Speech to text
  - id: keyboard
    custom:
      source: keyboard_op.py
      outputs:
        - buffer
        - submitted
        - record
        - ask
        - send
        - change
      inputs:
        recording: whisper/text

  - id: microphone
    operator:
      python: microphone_op.py
      inputs:
        record: keyboard/record
      outputs:
        - audio

  - id: whisper
    operator:
      python: whisper_op.py
      inputs:
        audio: microphone/audio
      outputs:
        - text

  ## Code Modifier
  - id: vectordb
    operator:
      python: sentence_transformers_op.py
      inputs:
        query: keyboard/change
        saved_file: file_saver/saved_file
      outputs:
        - raw_file

  - id: llm
    operator:
      python: llm_op.py
      inputs:
        code_modifier: vectordb/raw_file
        assistant: keyboard/ask
        message_sender: keyboard/send
      outputs:
        - modified_file
        - line
        - assistant_message

  - id: file_saver
    operator:
      python: file_saver_op.py
      inputs:
        file: llm/modified_file
      outputs:
        - saved_file
```

The keyboard, microphone, whisper node, works in a very similar fashion as the object detection dataflow and I'll let you check it out by yourself.

The code modification flow works by first comparing an instruction with a vectordb of operators source code and then feeding the most similar operator to an llm with the instruction for code modification.

The end result is then saved using the file saver
