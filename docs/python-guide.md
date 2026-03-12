# Getting Started with Python

This guide walks you through writing Python nodes and operators for adora dataflows.

## Prerequisites

```bash
pip install adora-rs-cli   # CLI (adora command)
pip install adora-rs       # Python node/operator API
```

The `adora-rs` package includes `pyarrow` as a dependency.

**Building from source** (instead of `pip install adora-rs`):

```bash
pip install maturin  # requires >= 1.8
cd apis/python/node && maturin develop --uv && cd ../../..
```

The operator API is compiled into the `adora` Python module automatically -- there is no separate install step for it.

## Hello World: Sender and Receiver

Create three files:

**sender.py** -- sends 100 numbered messages:

```python
import pyarrow as pa
from adora import Node

node = Node()
for i in range(100):
    node.send_output("message", pa.array([i]))
```

**receiver.py** -- receives and prints messages:

```python
from adora import Node

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
adora run dataflow.yml
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

All data flows through adora as Apache Arrow arrays. Common patterns:

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

Operators are lightweight alternatives to nodes. They run inside the adora runtime process (no separate OS process), making them faster for simple transformations.

Define an `Operator` class with an `on_event` method:

```python
# doubler_op.py
import pyarrow as pa
from adora import AdoraStatus

class Operator:
    def on_event(self, event, send_output) -> AdoraStatus:
        if event["type"] == "INPUT":
            value = event["value"].to_pylist()[0]
            send_output("doubled", pa.array([value * 2]), event["metadata"])
        return AdoraStatus.CONTINUE
```

Reference it in YAML with `operator` instead of `path`:

```yaml
nodes:
  - id: timer
    path: adora/timer/millis/500
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
from adora import Node

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

Use `node.log()` for structured logging that integrates with `adora logs`:

```python
node.log("info", "Processing item", {"count": str(i)})
```

Or use Python's standard `logging` module -- adora captures stdout/stderr automatically:

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
    path: adora/timer/millis/100    # tick every 100ms
    outputs:
      - tick

  - id: my-node
    path: my_node.py
    inputs:
      tick: tick-source/tick
```

Also available: `adora/timer/hz/30` for 30 Hz.

## Next Steps

- [Python API Reference](api-python.md) -- full API docs for Node, Operator, DataflowBuilder, CUDA
- [Communication Patterns](patterns.md) -- service (request/reply), action (goal/feedback/result), and streaming (session/segment/chunk) patterns
- [Examples](../examples/) -- python-dataflow, python-async, python-drain, python-concurrent-rw, python-multiple-arrays
- [Distributed Deployment](distributed-deployment.md) -- running across multiple machines with `adora up`
