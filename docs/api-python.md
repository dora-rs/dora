# Python API Reference

This document covers the Python APIs for building adora nodes, operators, and dataflows. Install with:

```bash
pip install adora-rs
```

---

## Table of Contents

- [Node API](#node-api)
  - [Node class](#node-class)
  - [Event dictionary](#event-dictionary)
  - [AdoraStatus enum](#adorastatus-enum)
- [Operator API](#operator-api)
  - [Operator class (user-defined)](#operator-class-user-defined)
  - [send_output callback](#send_output-callback)
- [DataflowBuilder](#dataflowbuilder)
  - [DataflowBuilder class](#dataflowbuilder-class)
  - [Node class (builder)](#node-class-builder)
  - [Output class (builder)](#output-class-builder)
  - [Operator class (builder)](#operator-class-builder)
- [CUDA Module](#cuda-module)
- [Quick Start Example](#quick-start-example)
- [DataflowBuilder Example](#dataflowbuilder-example)

---

## Node API

```python
from adora import Node
```

The `Node` class is the primary interface for custom nodes. It connects to a running dataflow, receives input events, and sends outputs.

### Node class

#### `__init__(node_id=None)`

Create a new node and connect to the running dataflow.

```python
# Standard: node ID is read from environment variables set by the daemon
node = Node()

# Dynamic: connect to a running dataflow by explicit node ID
node = Node(node_id="my-dynamic-node")
```

**Parameters:**
- `node_id` (str, optional) -- Explicit node ID for dynamic nodes. When omitted, the node reads its identity from environment variables set by the adora daemon.

**Raises:** `RuntimeError` if the node cannot connect to the dataflow.

---

#### `next(timeout=None)`

Retrieve the next event from the event stream. Blocks until an event is available or the timeout expires.

```python
event = node.next()              # block indefinitely
event = node.next(timeout=2.0)   # block up to 2 seconds
```

**Parameters:**
- `timeout` (float, optional) -- Maximum wait time in seconds.

**Returns:** `dict` -- An [event dictionary](#event-dictionary), or `None` if all senders have been dropped or the timeout expired.

---

#### `drain()`

Retrieve all buffered events without blocking.

```python
events = node.drain()
for event in events:
    print(event["type"])
```

**Returns:** `list[dict]` -- A list of [event dictionaries](#event-dictionary). Returns an empty list if no events are buffered.

---

#### `try_recv()`

Non-blocking receive. Returns the next buffered event if one is available.

```python
event = node.try_recv()
if event is not None:
    print(event["type"])
```

**Returns:** `dict | None` -- An [event dictionary](#event-dictionary), or `None` if no event is buffered.

---

#### `recv_async(timeout=None)`

Asynchronous receive. For use with `asyncio`.

```python
event = await node.recv_async()
event = await node.recv_async(timeout=5.0)
```

**Parameters:**
- `timeout` (float, optional) -- Maximum wait time in seconds. Returns an error if the timeout is reached.

**Returns:** `dict | None` -- An [event dictionary](#event-dictionary), or `None` if all senders have been dropped.

> **Note:** This method is experimental. The pyo3 async (Rust-Python FFI) integration is still in development.

---

#### `is_empty()`

Check whether there are any buffered events in the event stream.

```python
if not node.is_empty():
    event = node.try_recv()
```

**Returns:** `bool`

---

#### `send_output(output_id, data, metadata=None)`

Send data on an output channel.

```python
import pyarrow as pa

# Send raw bytes
node.send_output("status", b"OK")

# Send an Apache Arrow array (zero-copy capable)
node.send_output("values", pa.array([1, 2, 3]))

# Send with metadata
node.send_output("image", pa.array(pixels), {"camera_id": "front"})
```

**Parameters:**
- `output_id` (str) -- The output name as declared in the dataflow YAML.
- `data` (bytes | pyarrow.Array) -- The payload. Use `bytes` for simple data or `pyarrow.Array` for zero-copy shared-memory transport.
- `metadata` (dict, optional) -- Key-value pairs attached to the message. Supported value types: `bool`, `int`, `float`, `str`, `list[int]`, `list[float]`, `list[str]`, `datetime.datetime`.

**Raises:** `RuntimeError` if `data` is neither `bytes` nor a `pyarrow.Array`.

##### Service, action, and streaming patterns

Python nodes use the same metadata key conventions as Rust for [communication patterns](patterns.md). Parameters are plain dicts with string keys.

**Well-known metadata keys:**

| Key | Description |
|-----|-------------|
| `"request_id"` | Service request/response correlation (UUID v7) |
| `"goal_id"` | Action goal identification (UUID v7) |
| `"goal_status"` | Action result status: `"succeeded"`, `"aborted"`, or `"canceled"` |
| `"session_id"` | Streaming session identifier |
| `"segment_id"` | Streaming segment within a session (integer) |
| `"seq"` | Streaming chunk sequence number (integer) |
| `"fin"` | Last chunk of a streaming segment (bool) |
| `"flush"` | Discard older queued messages on input (bool) |

**Service client example:**

```python
import uuid

# Send a request with a unique request_id
request_id = str(uuid.uuid7())  # Python 3.13+; use uuid_utils or uuid.uuid4() on older versions
node.send_output("request", data, {"request_id": request_id})
```

**Service server example:**

```python
# Pass through the metadata (includes request_id) from the incoming request
node.send_output("response", result, event["metadata"])
```

**Action client example:**

```python
goal_id = str(uuid.uuid7())
node.send_output("goal", data, {"goal_id": goal_id})
```

**Streaming example** (flush downstream queues on user interruption):

```python
params = {
    "session_id": session_id,
    "segment_id": 1,
    "seq": 0,
    "fin": False,
    "flush": True,
}
node.send_output("text", data, metadata={"parameters": params})
```

See [patterns.md](patterns.md) for the full guide.

---

#### `log(level, message, target=None, fields=None)`

Emit a structured log message routed through the adora daemon.

```python
node.log("info", "Processing frame", target="vision")
node.log("error", "Sensor timeout", fields={"sensor": "lidar", "retry": "3"})
```

**Parameters:**
- `level` (str) -- Log level: `"error"`, `"warn"`, `"info"`, `"debug"`, or `"trace"`.
- `message` (str) -- The log message.
- `target` (str, optional) -- Target module or subsystem name.
- `fields` (dict[str, str], optional) -- Structured key-value context fields.

Works with the daemon's `min_log_level` filtering and `send_logs_as` routing.

---

#### `dataflow_descriptor()`

Return the full dataflow descriptor (the parsed dataflow YAML) as a Python dictionary.

```python
descriptor = node.dataflow_descriptor()
print(descriptor["nodes"])
```

**Returns:** `dict`

---

#### `node_config()`

Return the configuration block for this node from the dataflow descriptor.

```python
config = node.node_config()
model_path = config.get("model", "default.pt")
```

**Returns:** `dict`

---

#### `dataflow_id()`

Return the unique identifier of the running dataflow.

```python
print(node.dataflow_id())  # e.g. "a1b2c3d4-..."
```

**Returns:** `str`

---

#### `is_restart()`

Check whether this node was restarted after a previous exit or failure. Useful for deciding whether to restore saved state or start fresh.

```python
if node.is_restart():
    restore_checkpoint()
```

**Returns:** `bool`

---

#### `restart_count()`

Return how many times this node has been restarted. Returns `0` on the first run, `1` after the first restart, and so on.

```python
print(f"Restart #{node.restart_count()}")
```

**Returns:** `int`

---

#### `merge_external_events(subscription)`

Merge a ROS2 subscription stream into the node's main event loop. After calling this method, ROS2 messages arrive as events with `kind` set to `"external"`.

```python
from adora import Node, Ros2Context, Ros2Node, Ros2NodeOptions, Ros2Topic

node = Node()
ros2_context = Ros2Context()
ros2_node = ros2_context.new_node("listener", Ros2NodeOptions())
topic = Ros2Topic("/chatter", "std_msgs/String", ros2_node)
subscription = ros2_node.create_subscription(topic)

node.merge_external_events(subscription)

for event in node:
    if event["kind"] == "external":
        print("ROS2:", event["value"])
    elif event["type"] == "INPUT":
        print("Adora:", event["id"])
```

**Parameters:**
- `subscription` (`adora.Ros2Subscription`) -- A ROS2 subscription created via the adora ROS2 bridge.

---

#### Iteration support

The `Node` class implements `__iter__` and `__next__`, so you can iterate directly:

```python
for event in node:
    match event["type"]:
        case "INPUT":
            process(event["value"])
        case "STOP":
            break
```

The iterator calls `next()` with no timeout on each iteration. It yields `None` when the event stream is closed, which terminates the loop.

---

### Event dictionary

Events are returned as plain Python dictionaries. The structure depends on the event type.

#### INPUT

An input message arrived from another node.

```python
{
    "type": "INPUT",
    "id": "camera_image",          # input ID as declared in the dataflow YAML
    "kind": "adora",               # "adora" for dataflow events, "external" for ROS2
    "value": <pyarrow.Array>,      # the payload as an Apache Arrow array
    "metadata": {
        "timestamp": datetime,     # UTC-aware datetime.datetime
        "open_telemetry_context": "...",  # tracing context (if enabled)
        ...                        # any user-supplied metadata
    },
}
```

Access the data:
```python
values = event["value"].to_pylist()     # convert to Python list
array = event["value"].to_numpy()       # convert to NumPy array
```

#### INPUT_CLOSED

An input channel was closed (the upstream node finished).

```python
{
    "type": "INPUT_CLOSED",
    "id": "camera_image",
    "kind": "adora",
}
```

#### STOP

The dataflow is shutting down.

```python
{
    "type": "STOP",
    "id": "MANUAL" | "ALL_INPUTS_CLOSED",   # stop cause
    "kind": "adora",
}
```

#### ERROR

An error occurred in the runtime.

```python
{
    "type": "ERROR",
    "error": "description of the error",
    "kind": "adora",
}
```

#### External (ROS2)

When using `merge_external_events`, ROS2 messages arrive as:

```python
{
    "kind": "external",
    "value": <pyarrow.Array>,   # the ROS2 message as an Arrow array
}
```

---

### AdoraStatus enum

Used as the return value from operator `on_event` methods to control the event loop.

```python
from adora import AdoraStatus
```

| Value | Meaning |
|-------|---------|
| `AdoraStatus.CONTINUE` | Continue processing events (value `0`) |
| `AdoraStatus.STOP` | Stop this operator (value `1`) |
| `AdoraStatus.STOP_ALL` | Stop the entire dataflow (value `2`) |

---

## Operator API

Operators run inside the adora runtime process (no separate OS process). They are defined as a Python class named `Operator` with an `on_event` method.

### Operator class (user-defined)

Create a Python file with an `Operator` class:

```python
from adora import AdoraStatus

class Operator:
    def __init__(self):
        # Initialize state here
        self.count = 0

    def on_event(self, adora_event, send_output) -> AdoraStatus:
        if adora_event["type"] == "INPUT":
            self.count += 1
            # Process the input and optionally send output
            send_output("result", b"processed", adora_event["metadata"])
        return AdoraStatus.CONTINUE
```

**Methods:**
- `__init__(self)` -- Called once when the operator is loaded. Initialize any state or models here.
- `on_event(self, adora_event, send_output) -> AdoraStatus` -- Called for every incoming event. Must return an `AdoraStatus` value.

**Parameters of `on_event`:**
- `adora_event` (dict) -- An [event dictionary](#event-dictionary).
- `send_output` (callable) -- Callback to send output data (see below).

The runtime also sets `self.dataflow_descriptor` on the operator instance with the parsed dataflow YAML as a dictionary.

### send_output callback

The `send_output` callback is passed to `on_event` for sending data from an operator.

```python
send_output(output_id, data, metadata=None)
```

**Parameters:**
- `output_id` (str) -- The output name as declared in the dataflow YAML.
- `data` (bytes | pyarrow.Array) -- The payload.
- `metadata` (dict, optional) -- Metadata to attach. Pass `adora_event["metadata"]` to propagate tracing context.

**Example:**

```python
import pyarrow as pa
from adora import AdoraStatus

class Operator:
    def on_event(self, adora_event, send_output) -> AdoraStatus:
        if adora_event["type"] == "INPUT":
            result = pa.array([42], type=pa.int64())
            send_output("output", result, adora_event["metadata"])
        return AdoraStatus.CONTINUE
```

---

## DataflowBuilder

```python
from adora.builder import DataflowBuilder, Node, Operator, Output
```

Build dataflow YAML programmatically in Python.

### DataflowBuilder class

#### `__init__(name="adora-dataflow")`

Create a new dataflow builder.

```python
flow = DataflowBuilder("my-robot")
```

**Parameters:**
- `name` (str, optional) -- Name of the dataflow. Defaults to `"adora-dataflow"`.

#### `add_node(id, **kwargs) -> Node`

Add a node to the dataflow. Returns a `Node` object for further configuration.

```python
sender = flow.add_node("sender")
```

**Parameters:**
- `id` (str) -- Unique node identifier.
- `**kwargs` -- Additional node configuration passed through to the YAML.

**Returns:** `Node` (builder)

#### `to_yaml(path=None) -> str | None`

Generate the YAML representation of the dataflow. If `path` is given, writes to file and returns `None`. Otherwise returns the YAML string.

```python
# Write to file
flow.to_yaml("dataflow.yml")

# Get as string
yaml_str = flow.to_yaml()
```

**Parameters:**
- `path` (str, optional) -- File path to write the YAML.

**Returns:** `str | None`

#### Context manager

`DataflowBuilder` supports the `with` statement:

```python
with DataflowBuilder("my-flow") as flow:
    flow.add_node("sender").path("sender.py")
    flow.to_yaml("dataflow.yml")
```

---

### Node class (builder)

Returned by `DataflowBuilder.add_node()`. All setter methods return `self` for chaining.

#### `path(path) -> Node`

Set the path to the node's executable or script.

```python
node.path("my_node.py")
```

#### `args(args) -> Node`

Set command-line arguments for the node.

```python
node.args("--verbose --port 8080")
```

#### `env(env) -> Node`

Set environment variables for the node.

```python
node.env({"MODEL_PATH": "/models/yolo.pt"})
```

#### `build(command) -> Node`

Set the build command for the node (run before starting).

```python
node.build("pip install -r requirements.txt")
```

#### `git(url, branch=None, tag=None, rev=None) -> Node`

Set a Git repository as the source for the node.

```python
node.git("https://github.com/org/repo.git", branch="main")
```

#### `add_operator(operator) -> Node`

Attach an `Operator` to this node.

```python
op = Operator("detector", python="object_detection.py")
node.add_operator(op)
```

#### `add_output(output_id) -> Output`

Declare an output on this node and return an `Output` reference for use as an input source.

```python
output = sender.add_output("data")
```

#### `add_input(input_id, source, queue_size=None, queue_policy=None) -> Node`

Subscribe this node to an output from another node.

```python
# Using an Output object
output = sender.add_output("data")
receiver.add_input("data", output)

# Using a string reference
receiver.add_input("tick", "adora/timer/millis/100")

# With a custom queue size
receiver.add_input("images", camera_output, queue_size=2)

# Lossless input (blocks sender when full)
receiver.add_input("commands", cmd_output, queue_size=100, queue_policy="backpressure")
```

**Parameters:**
- `input_id` (str) -- Name of the input on this node.
- `source` (str | Output) -- Either a string (`"node_id/output_id"`) or an `Output` object.
- `queue_size` (int, optional) -- Maximum number of buffered messages for this input.
- `queue_policy` (str, optional) -- `"drop_oldest"` (default) or `"backpressure"` (buffers up to 10x `queue_size` before dropping).

#### `to_dict() -> dict`

Return the dictionary representation of the node for YAML serialization.

---

### Output class (builder)

Returned by `Node.add_output()`. Represents a reference to a node's output, used as a source in `add_input()`.

```python
output = sender.add_output("data")
receiver.add_input("sensor_data", output)
str(output)  # "sender/data"
```

---

### Operator class (builder)

Defines an operator for embedding in a node's YAML configuration.

#### `__init__(id, name=None, description=None, build=None, python=None, shared_library=None, send_stdout_as=None)`

```python
op = Operator(
    id="detector",
    python="object_detection.py",
    send_stdout_as="detection_text",
)
```

**Parameters:**
- `id` (str) -- Unique operator identifier.
- `name` (str, optional) -- Display name.
- `description` (str, optional) -- Human-readable description.
- `build` (str, optional) -- Build command to run before loading.
- `python` (str, optional) -- Path to the Python operator file.
- `shared_library` (str, optional) -- Path to a shared library operator.
- `send_stdout_as` (str, optional) -- Route the operator's stdout as an output with this ID.

#### `to_dict() -> dict`

Return the dictionary representation for YAML serialization.

---

## CUDA Module

```python
from adora.cuda import torch_to_ipc_buffer, ipc_buffer_to_ipc_handle, open_ipc_handle
```

Utilities for zero-copy GPU tensor sharing between nodes via CUDA IPC. Requires PyTorch with CUDA and Numba with CUDA support.

### `torch_to_ipc_buffer(tensor) -> tuple[pyarrow.Array, dict]`

Convert a PyTorch CUDA tensor into an Arrow array containing the CUDA IPC handle, plus a metadata dictionary. Send both through the dataflow to share GPU memory without copying.

```python
import torch
import pyarrow as pa
from adora import Node
from adora.cuda import torch_to_ipc_buffer

node = Node()
tensor = torch.randn(1024, 768, device="cuda")
ipc_buffer, metadata = torch_to_ipc_buffer(tensor)
node.send_output("gpu_data", ipc_buffer, metadata)
```

**Parameters:**
- `tensor` (torch.Tensor) -- A CUDA tensor.

**Returns:** `tuple[pyarrow.Array, dict]` -- The IPC handle as an int8 Arrow array, and metadata with shape, strides, dtype, size, offset, and source info.

---

### `ipc_buffer_to_ipc_handle(handle_buffer, metadata) -> IpcHandle`

Reconstruct a CUDA IPC handle from a received Arrow buffer and metadata.

```python
from adora.cuda import ipc_buffer_to_ipc_handle

event = node.next()
ipc_handle = ipc_buffer_to_ipc_handle(event["value"], event["metadata"])
```

**Parameters:**
- `handle_buffer` (pyarrow.Array) -- The Arrow array from `event["value"]`.
- `metadata` (dict) -- The metadata from `event["metadata"]`.

**Returns:** `numba.cuda.cudadrv.driver.IpcHandle`

---

### `open_ipc_handle(ipc_handle, metadata) -> ContextManager[torch.Tensor]`

Open a CUDA IPC handle and yield a PyTorch tensor. Use as a context manager to ensure proper cleanup.

```python
from adora.cuda import ipc_buffer_to_ipc_handle, open_ipc_handle

event = node.next()
ipc_handle = ipc_buffer_to_ipc_handle(event["value"], event["metadata"])

with open_ipc_handle(ipc_handle, event["metadata"]) as tensor:
    result = tensor * 2  # use the GPU tensor directly
```

**Parameters:**
- `ipc_handle` (`IpcHandle`) -- Handle from `ipc_buffer_to_ipc_handle`.
- `metadata` (dict) -- The metadata dictionary with shape, strides, and dtype info.

**Returns:** Context manager yielding a `torch.Tensor` on CUDA.

---

## Quick Start Example

A complete node that receives images, processes them, and sends results:

```python
#!/usr/bin/env python3
"""Example node: receives messages, transforms them, and sends output."""

import logging

import pyarrow as pa
from adora import Node


def main():
    node = Node()

    for event in node:
        if event["type"] == "INPUT":
            input_id = event["id"]

            if input_id == "message":
                values = event["value"].to_pylist()
                number = values[0]

                # Create a struct array with multiple fields
                result = pa.StructArray.from_arrays(
                    [
                        pa.array([number * 2]),
                        pa.array([f"Message #{number}"]),
                    ],
                    names=["doubled", "description"],
                )
                node.send_output("transformed", result)
                logging.info("Transformed message %d", number)

        elif event["type"] == "STOP":
            logging.info("Node stopping")
            break


if __name__ == "__main__":
    main()
```

Run with:

```bash
adora run dataflow.yml
```

---

## DataflowBuilder Example

Build a dataflow programmatically instead of writing YAML by hand:

```python
#!/usr/bin/env python3
"""Build a simple sender -> receiver dataflow."""

from adora.builder import DataflowBuilder, Operator

flow = DataflowBuilder("example-flow")

# Add a timer-driven sender node
sender = flow.add_node("sender")
sender.path("sender.py")
tick_output = sender.add_output("message")

# Add a receiver that subscribes to the sender
receiver = flow.add_node("receiver")
receiver.path("receiver.py")
receiver.add_input("message", tick_output)

# Add a node with a timer input
timed_node = flow.add_node("periodic")
timed_node.path("periodic.py")
timed_node.add_input("tick", "adora/timer/millis/100")

# Add a node with an operator
runtime_node = flow.add_node("runtime-node")
op = Operator("detector", python="object_detection.py")
runtime_node.add_operator(op)
runtime_node.add_input("image", "camera/image")

# Write or print the YAML
flow.to_yaml("dataflow.yml")
print(flow.to_yaml())
```
