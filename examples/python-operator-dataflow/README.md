# Python Dataflow Example

This examples shows how to create and connect dora operators and custom nodes in Python.

## Overview

The [`dataflow.yml`](./dataflow.yml) defines a simple dataflow graph with the following three nodes:

- a webcam node, that connects to your webcam and feed the dataflow with webcam frame as jpeg compressed bytearray.
- an object detection node, that apply Yolo v5 on the webcam image. The model is imported from Pytorch Hub. The output is the bouding box of each object detected, the confidence and the class. You can have more info here: https://pytorch.org/hub/ultralytics_yolov5/
- a window plotting node, that will retrieve the webcam image and the Yolov5 bounding box and join the two together.

## Getting started

```bash
cargo run --example python-dataflow 
```

## Installation

To install, you should run the `install.sh` script.

```bash
install.sh
```

## Run the dataflow as a standalone

- Start the `dora-coordinator`:

```
../../target/release/dora-daemon --run-dataflow dataflow.yml
```
