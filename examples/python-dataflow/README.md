# Python Dataflow Example

This example shows how to create and connect dora nodes in Python. It is a good starting point for understanding
how data flows between nodes in a dora application.

## Overview

The [`dataflow.yml`](./dataflow.yml) defines a simple dataflow graph with three nodes:

- **sender** — sends 100 integer messages (0 to 99), one every 100ms, then exits.
- **transformer** — receives each integer and produces a struct with three fields: the value doubled, a string label, and an even/odd flag.
- **receiver** — receives both the raw integer from `sender` and the transformed struct from `transformer`, and logs them.

```
sender ──► transformer ──► receiver
  │                            ▲
  └────────────────────────────┘
```

Both `sender/message` and `transformer/transformed` are wired to `receiver` as separate inputs.

### Dynamic node variant

[`dataflow_dynamic.yml`](./dataflow_dynamic.yml) is a separate example that uses a webcam (`opencv-video-capture`)
and a dynamic plot node (`opencv-plot`). It requires a connected webcam. See the
[Getting started (dynamic)](#getting-started-dynamic-node) section below.

## Getting started

After installing `dora-cli` and `uv`, install the Python node API and build the dataflow:

```bash
cd examples/python-dataflow
uv pip install -e ../../apis/python/node --reinstall
dora build ./dataflow.yml --uv
```

Then run the dataflow:

```bash
dora run ./dataflow.yml --uv
```

The dataflow runs until `sender` finishes sending all 100 messages and the nodes receive a `STOP` event.
Logs are written to `out/<session-id>/log_<node>.txt`.

## Expected output

**sender** (log_sender.txt):
```
INFO Sent message 0
INFO Sent message 1
...
INFO Sent message 99
INFO Sender finished
```

**transformer** (log_transformer.txt):
```
INFO Transformed message 0 -> struct with doubled=0
INFO Transformed message 1 -> struct with doubled=2
INFO Transformed message 2 -> struct with doubled=4
...
```

**receiver** (log_receiver.txt):
```
INFO Received message: [0]
INFO Received transformed: [{'doubled': 0, 'description': 'Message #0', 'is_even': True}]
INFO Received message: [1]
INFO Received transformed: [{'doubled': 2, 'description': 'Message #1', 'is_even': False}]
...
```

## Getting started (dynamic node)

`dataflow_dynamic.yml` requires a webcam. Build and start it with:

```bash
dora build ./dataflow_dynamic.yml --uv
dora start ./dataflow_dynamic.yml --uv
```

Then manually start the `opencv-plot` dynamic node in a second terminal:

```bash
# Make sure your virtual environment is activated
python -m opencv_plot
```
