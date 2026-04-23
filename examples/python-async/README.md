# Python Async

Demonstrates async/await patterns for Python nodes using `node.recv_async()` instead of blocking iteration.

## Architecture

```
timer (10ms) --> send_data --> data --> receive_data_with_sleep (async)
```

## Nodes

**send_data** (`send_data.py`) -- Sends 100 messages containing uint64 nanosecond timestamps on each timer tick. Uses the standard synchronous `for event in node` loop.

**receive_data_with_sleep** (`receive_data.py`) -- Uses `await node.recv_async()` inside an async function to receive events without blocking the asyncio event loop. Processes 50 events then exits.

```python
async def main():
    node = Node()
    for _ in range(50):
        event = await node.recv_async()
        if event["type"] == "STOP":
            break
```

## When to Use Async

Use `recv_async()` when your node needs to:
- Make HTTP/gRPC calls alongside event processing
- Run multiple concurrent tasks (e.g., `asyncio.gather`)
- Integrate with async libraries (aiohttp, asyncpg, etc.)

For simple sequential processing, the synchronous `for event in node` loop is simpler.

## Run

```bash
dora run dataflow.yaml
```

## What This Demonstrates

| Feature | Where |
|---------|-------|
| `await node.recv_async()` | Receiver |
| asyncio event loop integration | Receiver |
| Mixed sync sender + async receiver | Both nodes |
