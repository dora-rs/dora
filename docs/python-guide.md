# Getting Started with Python

This guide walks you through writing Python nodes and operators for dora dataflows.

## Prerequisites

```bash
cargo install dora-cli    # CLI (dora command)
pip install dora-rs       # Python node/operator API
```

The `dora-rs` package includes `pyarrow` as a dependency.

**Building from source** (instead of `pip install dora-rs`):

```bash
pip install maturin  # requires >= 1.8
cd apis/python/node && maturin develop --uv && cd ../../..
```

The operator API is compiled into the `dora` Python module automatically -- there is no separate install step for it.

## Hello World: Sender and Receiver

Create three files:

**sender.py** -- sends 100 numbered messages:

```python
import pyarrow as pa
from dora import Node

node = Node()
for i in range(100):
    node.send_output("message", pa.array([i]))
```

**receiver.py** -- receives and prints messages:

```python
from dora import Node

node = Node()
for event in node:
    if event["type"] == "INPUT":
        values = event["value"].to_pylist()
        print(f"Received {event['id']}: {values}")
    elif event["type"] == "STOP":
        break
```

**dataflow.yml** -- connects sender to receiver:

```yaml
nodes:
  - id: sender
    path: sender.py
    outputs:
      - message

  - id: receiver
    path: receiver.py
    inputs:
      message: sender/message
```

Run it:

```bash
dora run dataflow.yml
```

## Events

Every call to `node.next()` or iteration over `for event in node` returns an event dictionary:

| Key | Type | Description |
|-----|------|-------------|
| `type` | str | `"INPUT"`, `"INPUT_CLOSED"`, `"STOP"`, or `"ERROR"` |
| `id` | str | Input name (e.g. `"message"`) -- only for `INPUT` events |
| `value` | pyarrow.Array or None | The data payload |
| `metadata` | dict | Tracing/routing metadata |

Handle events by checking `event["type"]`:

```python
for event in node:
    match event["type"]:
        case "INPUT":
            process(event["id"], event["value"])
        case "INPUT_CLOSED":
            print(f"Input {event['id']} closed")
        case "STOP":
            break
```

## Working with Arrow Data

All data flows through dora as Apache Arrow arrays. Common patterns:

```python
import pyarrow as pa

# Simple values
node.send_output("count", pa.array([42]))
node.send_output("names", pa.array(["alice", "bob"]))

# Read values back
values = event["value"].to_pylist()  # [42] or ["alice", "bob"]

# Structured data
struct = pa.StructArray.from_arrays(
    [pa.array([1.5]), pa.array(["hello"])],
    names=["x", "y"],
)
node.send_output("point", struct)

# Raw bytes (images, serialized data, etc.)
node.send_output("frame", pa.array(raw_bytes))
```

## Operators

Operators are lightweight alternatives to nodes. They run inside the dora runtime process (no separate OS process), making them faster for simple transformations.

Define an `Operator` class with an `on_event` method:

```python
# doubler_op.py
import pyarrow as pa
from dora import DoraStatus

class Operator:
    def on_event(self, event, send_output) -> DoraStatus:
        if event["type"] == "INPUT":
            value = event["value"].to_pylist()[0]
            send_output("doubled", pa.array([value * 2]), event["metadata"])
        return DoraStatus.CONTINUE
```

Reference it in YAML with `operator` instead of `path`:

```yaml
nodes:
  - id: timer
    path: dora/timer/millis/500
    outputs:
      - tick

  - id: doubler
    operator:
      python: doubler_op.py
      inputs:
        tick: timer/tick
      outputs:
        - doubled
```

**When to use operators vs nodes:**

| | Nodes | Operators |
|--|-------|-----------|
| Process model | Separate OS process | In-process (shared runtime) |
| Startup cost | Higher | Lower |
| Isolation | Full process isolation | Shared memory space |
| Best for | Long-running, heavy compute | Lightweight transforms, filters |

## Async Nodes

For nodes that need async I/O (HTTP calls, database queries, etc.), use `recv_async()`:

```python
import asyncio
from dora import Node

async def main():
    node = Node()
    for _ in range(50):
        event = await node.recv_async()
        if event["type"] == "STOP":
            break
        # Do async work here
        result = await fetch_data(event["value"])
        node.send_output("result", result)

asyncio.run(main())
```

See [examples/python-async](../examples/python-async) for a complete example.

## Logging

Use `node.log()` for structured logging that integrates with `dora logs`:

```python
node.log("info", "Processing item", {"count": str(i)})
```

Or use Python's standard `logging` module -- dora captures stdout/stderr automatically:

```python
import logging
logging.info("Processing item %d", i)
```

See [examples/python-logging](../examples/python-logging) for logging module integration.

## Timers

Built-in timer nodes generate periodic ticks without writing any code:

```yaml
nodes:
  - id: tick-source
    path: dora/timer/millis/100    # tick every 100ms
    outputs:
      - tick

  - id: my-node
    path: my_node.py
    inputs:
      tick: tick-source/tick
```

Also available: `dora/timer/hz/30` for 30 Hz.

## Reproducible Dependencies with `--uv`

For Python nodes with a `build:` block, pass `--uv` to `dora build` / `dora start` / `dora run`:

```yaml
- id: vision
  path: vision.py
  build: pip install -r requirements.txt
```

```bash
dora build --uv dataflow.yml
dora run   --uv dataflow.yml
```

With `--uv`, dora creates a dedicated `uv` virtual environment at `<working-dir>/.dora/python-envs/<node-id>/` for each Python node, installs your `build:` deps into it, and reuses that same interpreter when the node is spawned. This gives you:

- **Build-time deps == runtime deps.** No drift if the system Python changes between `dora build` and `dora run`.
- **No cross-node contamination.** A node with `tensorflow==2.10` and a node with `pytorch` in the same dataflow each get their own venv.
- **No cross-dataflow contamination.** Building dataflow A with `numpy 1.x` does not overwrite dataflow B's `numpy 2.x`.
- **Hermetic subprocesses.** Anything your node spawns (`subprocess.run(["pip", ...])`, console scripts, `python -m pip`) resolves to the managed env, not the ambient one.

Script-only Python nodes (no `build:` block) keep using your active `uv` environment as before — they have no deps for dora to install. See [CLI reference](cli.md#dora-build) for details and the [`dora doctor` guide](debugging.md#environment-diagnosis) to verify `uv` is on PATH.

## Zero-Copy Sends with `send_output_raw`

`node.send_output(...)` performs **one copy** to move your data into dora's send buffer. For large payloads (camera frames, point clouds, embeddings) at high rates that copy is a real CPU + memory-bandwidth cost.

`node.send_output_raw(output_id, length, metadata=...)` returns a `SampleHandler` that owns dora's pre-allocated send buffer. You write into it directly via Python's buffer protocol (`memoryview`, `numpy.asarray`, `struct.pack_into`) and call `.send()` — no intermediate copy.

**Requires Python >= 3.11** (uses the stable buffer-protocol C API).

### Recommended: context-manager form

```python
height, width = 1080, 1920
with node.send_output_raw("frame", height * width * 3, metadata={"width": width}) as buf:
    np.asarray(buf, dtype=np.uint8).reshape(height, width, 3)[:] = frame
# data is sent automatically when the `with` block exits
```

### Manual form (when send timing is conditional)

```python
sample = node.send_output_raw("frame", height * width * 3)
mv = sample.as_memoryview()
arr = np.asarray(mv, dtype=np.uint8).reshape(height, width, 3)
arr[:] = frame
del arr        # drop derived numpy view first
mv.release()   # then release the memoryview itself
sample.send()  # ship it
```

### Safety contract

The handler enforces a small protocol so the zero-copy path stays sound:

- **Write-only-once.** Calling `.send()` twice on the same handler is an error.
- **No view past `send()`.** Once `.send()` runs, any subsequent attempt to acquire a buffer view raises `BufferError`.
- **No `send()` while views are open.** If you still hold a `memoryview` (or a derived numpy array that hasn't been GC'd), `.send()` errors with a clear message. Release the views first.

The context-manager form handles this automatically — `__exit__` releases the cached memoryview before calling send. The manual form requires explicit `del` of any derived arrays + `mv.release()` before `sample.send()`.

See [`examples/python-zero-copy-send/`](https://github.com/dora-rs/dora/tree/main/examples/python-zero-copy-send) for a runnable example.

## Next Steps

- [Python API Reference](api-python.md) -- full API docs for Node, Operator, DataflowBuilder, CUDA
- [Communication Patterns](patterns.md) -- service (request/reply), action (goal/feedback/result), and streaming (session/segment/chunk) patterns
- [Examples](../examples/) -- python-dataflow, python-async, python-drain, python-concurrent-rw, python-multiple-arrays
- [Distributed Deployment](distributed-deployment.md) -- running across multiple machines with `dora up`
