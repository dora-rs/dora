# Python Dataflow Example

This examples shows how to create and connect dora nodes in Python.

## Overview

The [`dataflow.yml`](./dataflow.yml) defines a simple dataflow graph with the following three nodes:

- a webcam node, that connects to your webcam and feed the dataflow with webcam frame as jpeg compressed bytearray.
- an object detection node, that apply Yolo v5 on the webcam image. The model is imported from Pytorch Hub. The output
  is the bounding box of each object detected, the confidence and the associated label. You can have more info
  here: https://pytorch.org/hub/ultralytics_yolov5/
- a window plotting node, that will retrieve the webcam image and the Yolov5 bounding box and join the two together.

The same dataflow is implemented for a `dynamic-node` in [`dataflow_dynamic.yml`](./dataflow_dynamic.yml). It contains
the same nodes as the previous dataflow, but the object detection node is a dynamic node. See the next section for more
information on how to start such a dataflow.

## Getting started

After installing Rust, `dora-cli` and `Python >3.11`, you will need to **activate** (or create and **activate**) a
virtual
environment. Then, you will need to install the dependencies:

```bash
cd examples/python-dataflow
dora build ./dataflow.yml (or dora build ./dataflow_dynamic.yml)
```

It will install the required dependencies for the Python nodes.

Then you can run the dataflow:

```bash
dora up
dora start ./dataflow.yml (or dora start ./dataflow_dynamic.yml)
```

**Note**: if you're running the dynamic dataflow, you will need to start manually the ultralytics-yolo node:

```bash
# activate your virtual environment in another terminal
python ultralytics-yolo --name object-detection --model yolov5n.pt
```