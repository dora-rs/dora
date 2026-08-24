# Python Serial Event Loop

A task-responsive event-loop framework for Dora Python nodes. It provides a
dedicated `SerialWorker` for each registered input ID, timer management, and
thread-safe output delivery while preserving Dora's native Apache Arrow
values.

The main loop polls Dora events on one thread. Each input is quickly placed in
the FIFO queue of its dedicated worker, allowing the main loop to continue
receiving events without waiting for application processing to finish.

## Requirements

- Python 3.10 or later
- The Dora Python package
- PyArrow

## Files

- `serial_event_loop.py`: framework implementation
- `test_serial_event_loop.py`: unit tests using a fake Dora node

## Example

```python
import pyarrow as pa

from serial_event_loop import SerialEventLoop


loop = SerialEventLoop("my_node")


def handle_camera(event):
    # event.data is the native Arrow value supplied by Dora.
    loop.send_output("processed_result", event.data, event.metadata)


loop.register_handler("camera", handle_camera)
loop.register_timer(
    "heartbeat",
    1.0,
    lambda: loop.send_output("heartbeat", pa.array([True])),
)
loop.run()
```

## API Overview

### `SerialWorker`

Each worker owns a thread and a FIFO queue. Calls to `enqueue()` return without
waiting for the handler, while events accepted by a worker are processed
strictly in enqueue order. `stop()` drains accepted events and joins the
worker thread.

### `register_handler(id, handler)`

Registers a dedicated worker for a Dora input ID. Registering the same ID
again replaces and stops the previous worker. Handlers for different input IDs
may run concurrently, but events for the same ID are always processed
serially.

The handler receives an `InputEvent` with these attributes:

- `id`: Dora input ID
- `data`: native Arrow value from the Dora event
- `metadata`: Dora event metadata

### `register_timer(id, interval, handler, repeat=True)`

Registers or replaces a timer. Numeric intervals are expressed in seconds;
`datetime.timedelta` values are also accepted. Set `repeat=False` for a
one-shot timer.

```python
from datetime import timedelta

loop.register_timer("fast_tick", 0.1, publish_status)
loop.register_timer(
    "delayed_init",
    timedelta(seconds=5),
    late_init,
    repeat=False,
)
```

### `cancel_timer(id)`

Cancels a registered timer. It returns `True` when the timer existed and was
removed, or `False` when no matching timer was registered.

### `send_output(output_id, data, metadata=None)`

Queues a Dora output safely from any thread. Input handlers and timer
callbacks can call this method directly; the main event-loop thread performs
the actual `Node.send_output()` call.

### `run()`

Starts the blocking Dora event loop. Polling timeouts do not stop it; the loop
continues until Dora emits a `STOP` event or another thread calls `close()`.
Each `SerialEventLoop` instance can call `run()` only once.

Dora 0.4.1 reports polling timeouts as `ERROR` events. The framework silently
retries only errors beginning with `Timeout event stream error:`; other Dora
errors are logged at error level so real runtime failures remain visible.

### `close()`

Stops timers and drains all registered workers. Call it directly when using
only workers or timers without subsequently calling `run()`. Normal `run()`
shutdown performs this cleanup automatically.

## Run the Tests

The unit tests do not require a live Dora runtime:

```bash
cd python-serial-event-loop
python3 -m unittest -v test_serial_event_loop.py
```

## Example Dataflow

The included dataflow runs two Python Dora nodes:

```text
dora/timer/millis/100 -> sender -> counter -> receiver
```

The sender publishes an incrementing Arrow `uint64` counter at 10 Hz. The
receiver processes each value through `SerialEventLoop` and prints output such
as:

```text
[python-receiver] received counter=0
[python-receiver] received counter=1
```

From this directory, run the example. Press `Ctrl+C` to stop it:

```bash
dora run dataflow.yml
```

Run both the framework and example-node unit tests with:

```bash
python3 -m unittest -v test_serial_event_loop.py test_example_nodes.py
```
