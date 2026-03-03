# Python Concurrent Read/Write

Demonstrates multi-threaded concurrent operation within a single Python node: one thread reads events while another publishes independently.

## Architecture

```
node1 --> data --> node2
node2 --> data --> node1   (circular)
```

Two identical nodes form a circular dataflow. Each node runs a reader thread and a daemon publisher thread concurrently.

## Nodes

**node1 / node2** (`receive_data.py`) -- Each node spawns two threads:
- **Reader thread**: calls `node.next()` in a blocking loop, processes incoming events, prints received values
- **Publisher thread** (daemon): sends `pa.array([timestamp])` every second on the `data` output

The publisher runs as a daemon thread so it automatically terminates when the reader finishes.

## Run

```bash
adora run dataflow.yml
```

## What This Demonstrates

| Feature | Where |
|---------|-------|
| `threading.Thread` for concurrent node I/O | Both nodes |
| Daemon threads for background publishing | Publisher thread |
| `node.next()` blocking receive | Reader thread |
| Circular dataflow (bidirectional) | YAML |
| `time.perf_counter_ns()` for timestamps | Publisher thread |
