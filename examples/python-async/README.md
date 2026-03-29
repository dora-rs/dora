# Python Async Example

This example shows how to use Python's `asyncio` with dora nodes. Instead of the
blocking synchronous API (`for event in node:` or `node.next()`), nodes can use
`await node.recv_async()` to receive events without blocking the thread — allowing
other coroutines to run concurrently.

Use the async API when your node needs to integrate with async frameworks (e.g.
`asyncio`, web servers, async I/O libraries).

## Overview

The [`dataflow.yaml`](./dataflow.yaml) defines two nodes:

- **send_data** — triggered every 10ms by a built-in dora timer, sends the current
  timestamp (nanoseconds, `uint64`) as output `data`. Stops after 100 sends.
- **receive_data_with_sleep** — receives each timestamp using `await node.recv_async()`
  inside an `asyncio` event loop. Processes 50 events, then prints `done!` and exits.

```
dora/timer/millis/10 ──► send_data ──► receive_data_with_sleep
```

### Sync vs async API

| | Sync | Async |
|---|---|---|
| Receive next event | `for event in node:` or `node.next()` | `await node.recv_async()` |
| Thread behaviour | Blocks until event arrives | Yields to event loop while waiting |
| Use when | Simple scripts | Integrating with asyncio / async frameworks |

## Getting started

Install dependencies and build the dataflow:

```bash
cd examples/python-async
uv venv --seed -p 3.11
uv pip install -e ../../apis/python/node
dora build dataflow.yaml --uv
```

Run the dataflow:

```bash
dora run dataflow.yaml --uv
```

## Expected output

The `receive_data_with_sleep` node prints a single line after processing 50 events:

```
done!
```

No output is produced by `send_data` — it sends timestamps silently and exits after
100 sends.
