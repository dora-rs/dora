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
conda create -n example_env python=3.11
conda activate test_env
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

The keyboard, microphone, whisper node, works in a very similar fashion as the object detection dataflow and I'll let you check it out by yourself.

The code modification flow works by first comparing an instruction with a vectordb of operators source code and then feeding the most similar operator to an llm with the instruction for code modification.

The end result is then saved using a file saver.
