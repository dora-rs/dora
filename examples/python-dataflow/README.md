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
| `dataflow.yml` | Standard sender/transformer/receiver pipeline. Script-only Python nodes (no `build:` blocks) — runs against your ambient Python. |
| `dataflow_dynamic.yml` | Camera + opencv-plot vision pipeline with dynamic node loading. Each node has its own `build:` line (different `git+` install per node), so it's the natural fit for the `--uv` managed-env flow below. |

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

Or pass `--uv` to run the Python nodes through `uv` instead of the system Python:

```bash
dora run dataflow.yml --uv
```

For `dataflow.yml` (script-only nodes, no `build:` blocks) this just routes spawns through `uv run python` against your active `uv` env — there are no per-node deps for dora to install. See the next section for the case where `--uv` actually creates per-node managed envs.

Expected output (receiver logs):

```
Received message: [0]
Received transformed: [{'doubled': 0, 'description': 'Message #0', 'is_even': True}]
Received message: [1]
Received transformed: [{'doubled': 2, 'description': 'Message #1', 'is_even': False}]
...
```

## Per-node managed environments (`--uv` + `build:`)

When a Python node has a `build:` block, `--uv` switches into a different mode: dora creates a dedicated `uv` venv per node and installs that node's `build:` line into it. The runtime spawns each node against its own interpreter, so build-time deps == runtime deps and no two nodes share `site-packages`.

`dataflow_dynamic.yml` is the demo: `camera` installs `opencv-video-capture` and `opencv-plot` installs `opencv-plot` — two different `pip install` lines, two different sets of deps.

```bash
dora build --uv dataflow_dynamic.yml
dora run   --uv dataflow_dynamic.yml
```

After `dora build --uv`, the working directory contains:

```
.dora/python-envs/
  camera/         <- isolated venv with opencv-video-capture installed
  opencv-plot/    <- isolated venv with opencv-plot installed
```

`dora run --uv` then spawns each node against its own `.dora/python-envs/<node-id>/bin/python` (or `Scripts/python.exe` on Windows), with `VIRTUAL_ENV` set so any `subprocess.run(["pip", ...])` / console scripts / `python -m pip` inside the node also resolve from the managed env.

Run `dora doctor` to confirm `uv` is on `PATH` before trying this flow. See the [CLI reference](../../docs/cli.md#dora-build) for the full description of the `--uv` build/runtime contract.

## What This Demonstrates

| Feature | Where |
|---------|-------|
| `pa.array()` for simple values | Sender |
| `pa.StructArray` for structured data | Transformer |
| Multiple inputs on one node | Receiver (message + transformed) |
| Event type handling (INPUT, STOP) | Receiver |
| `event["id"]` to distinguish inputs | Receiver |
| `event["value"].to_pylist()` for reading | Receiver |
