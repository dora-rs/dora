# Python Dataflow Example

This examples shows how to create and connect dora operators and custom nodes in Python.

## Overview

The [`dataflow.yml`](./dataflow.yml) defines a simple dataflow graph with the following three nodes:

- a webcam node, that connects to your webcam and feeds the dataflow with raw webcam frames as flattened numpy arrays.
- an object detection node, that applies YOLOv8 (via the `ultralytics` package) on the webcam image. The output is the bounding box of each object detected, the confidence and the class.
- a window plotting node, that will retrieve the webcam image and the Yolov5 bounding box and join the two together.

## Getting started

```bash
pip install -r requirements.txt
cargo run --example python-operator-dataflow
```

## Installation

```bash
conda create -n example_env python=3.11
conda activate example_env
pip install -r requirements.txt
pip install -r requirements_llm.txt
```

### Alternative: managed venvs with `--uv`

Since each operator's `build:` line in [`dataflow.yml`](./dataflow.yml) is `pip install -r requirements.txt`, you can let dora manage per-operator [`uv`](https://github.com/astral-sh/uv) virtual environments instead of running `conda` by hand:

```bash
dora build --uv dataflow.yml
dora run   --uv dataflow.yml
```

`dora build --uv` creates one venv per operator at `<dataflow-dir>/.dora/python-envs/<node-id>/`, runs the operator's `build:` command inside it, and records the env path. `dora run --uv` then spawns each operator against that exact interpreter — same Python, same deps as build time, no cross-operator contamination. Run `dora doctor` to confirm `uv` is on `PATH` before trying this flow.

This is the closest analogue to the per-operator [`dataflow_conda.yml`](./dataflow_conda.yml) pattern (which uses `conda_env:` on a single operator). Use whichever fits your environment: `conda_env:` when you already have conda envs in place and want explicit naming; `--uv` when you want dora to provision the env from `pip install` build steps automatically. The two are not mutually exclusive at the operator level — an operator without `conda_env:` falls back to the `--uv` managed env when `--uv` is passed.

## Run the dataflow

- Start the object detection dataflow alone:

```bash
dora run dataflow.yml
```

- Start the llm dataflow (Only works on Windows and Linux):

```bash
dora run dataflow_llm.yml
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
