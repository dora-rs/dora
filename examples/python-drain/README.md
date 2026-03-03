# Python Drain

Demonstrates queue draining and non-blocking receive -- useful for clearing buffered messages or polling without blocking.

## Architecture

```
timer (10ms) --> send_data --> data --> receive_data_with_sleep
```

## Nodes

**send_data** (`send_data.py`) -- Sends 100 messages containing uint64 nanosecond timestamps at 10ms intervals. The high-frequency sender fills the receiver's input queue while the receiver sleeps.

**receive_data_with_sleep** (`receive_data.py`) -- Sleeps 1 second (allowing ~100 messages to queue up), then calls `node.drain()` to consume all pending events at once. After draining, loops 100 times calling `node.try_recv()` for non-blocking receive with error handling.

## Key API Methods

- **`node.drain()`** -- returns all queued events immediately without blocking
- **`node.try_recv()`** -- returns one event if available, raises error if empty (non-blocking)
- **`queue_size: 100`** -- YAML setting that controls input buffer capacity

## Run

```bash
adora run dataflow.yaml
```

## What This Demonstrates

| Feature | Where |
|---------|-------|
| `node.drain()` for batch consumption | Receiver |
| `node.try_recv()` for non-blocking poll | Receiver |
| `queue_size` for input buffer sizing | YAML (both nodes) |
| High-frequency producer with slow consumer | Sender vs receiver |
