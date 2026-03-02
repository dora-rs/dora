# Python Dataflow Example

This example shows how to create and connect adora nodes in Python.

## Overview

The [`dataflow.yml`](./dataflow.yml) defines a simple dataflow graph with three nodes:

- **sender** (`sender.py`): Produces messages on the `message` output
- **transformer** (`transformer.py`): Receives messages from the sender and outputs `transformed` data
- **receiver** (`receiver.py`): Consumes both the original `message` from the sender and `transformed` data from the transformer

A dynamic node variant is available in [`dataflow_dynamic.yml`](./dataflow_dynamic.yml).

## Getting started

After installing Rust, `adora-cli` and `uv` (if you installed the cli without pip), install the dependencies:

```bash
cd examples/python-dataflow
uv pip install -e ../../apis/python/node --reinstall
adora build ./dataflow.yml --uv
```

Then run the dataflow:

```bash
adora run ./dataflow.yml --uv
```

For the dynamic dataflow variant:

```bash
adora build ./dataflow_dynamic.yml --uv
adora run ./dataflow_dynamic.yml --uv
```
