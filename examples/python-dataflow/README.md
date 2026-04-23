# Python Dataflow

Basic three-node Python pipeline: sender produces messages, transformer enriches them, receiver consumes both streams.

## Architecture

```
sender --> message     --> receiver
        -> message     --> transformer --> transformed --> receiver
```

The receiver has two inputs: raw `message` from sender and `transformed` from transformer.

## Nodes

**sender** (`sender.py`) -- Sends 100 numbered messages as `pa.array([i])` with 100ms delays. Exits after the loop completes.

**transformer** (`transformer.py`) -- Receives each message number and creates a `StructArray` with three fields:
- `doubled`: the value times 2
- `description`: a formatted string (`"Message #N"`)
- `is_even`: boolean flag

**receiver** (`receiver.py`) -- Iterates over events, distinguishing inputs by `event["id"]`:
- `"message"`: prints the raw integer array
- `"transformed"`: prints the struct fields
- Breaks on `STOP` event

## Variants

| File | Description |
|------|-------------|
| `dataflow.yml` | Standard sender/transformer/receiver pipeline |
| `dataflow_dynamic.yml` | Separate example: camera + opencv-plot vision pipeline with dynamic node loading |

## Prerequisites

Install the Python node API (the PyPI package is `dora-rs`, **not** `dora`):

```bash
pip install dora-rs
```

> **Note:** The Python import name is `dora` (`from dora import Node`), but the
> PyPI package name is **`dora-rs`**. Running `pip install dora` installs an
> unrelated package and will cause `ImportError: cannot import name 'Node'`.

## Run

```bash
dora run dataflow.yml
```

Or use `uv` to manage the Python environment automatically:

```bash
dora run dataflow.yml --uv
```

Expected output (receiver logs):

```
Received message: [0]
Received transformed: [{'doubled': 0, 'description': 'Message #0', 'is_even': True}]
Received message: [1]
Received transformed: [{'doubled': 2, 'description': 'Message #1', 'is_even': False}]
...
```

## What This Demonstrates

| Feature | Where |
|---------|-------|
| `pa.array()` for simple values | Sender |
| `pa.StructArray` for structured data | Transformer |
| Multiple inputs on one node | Receiver (message + transformed) |
| Event type handling (INPUT, STOP) | Receiver |
| `event["id"]` to distinguish inputs | Receiver |
| `event["value"].to_pylist()` for reading | Receiver |
