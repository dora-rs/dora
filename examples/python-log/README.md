# Python Log

Demonstrates Python `logging` module integration with adora's log capture system.

## Architecture

```
timer (10ms) --> send_data --> data --> receive_data_with_sleep (logging)
```

## Nodes

**send_data** (`send_data.py`) -- Sends 100 uint64 nanosecond timestamps on each timer tick.

**receive_data_with_sleep** (`receive_data.py`) -- Receives events and logs at multiple levels using Python's standard `logging` module:

```python
node = Node()
log = logging.getLogger(__name__)

for event in node:
    if event["type"] == "INPUT":
        logging.info(f"info {event['value'].to_numpy()}")
        log.log(logging.DEBUG, f"received {event['id']} with data")
        log.warning("THIS IS A WARNING")
    if event["type"] == "ERROR":
        log.log(logging.ERROR, f"received {event} with data")
```

Adora captures Python logging output automatically -- no special configuration needed. Use `--log-level` to filter:

```bash
adora run dataflow.yaml --log-level warn    # only warnings and above
adora run dataflow.yaml --log-level debug   # everything
```

## Run

```bash
adora run dataflow.yaml
```

## What This Demonstrates

| Feature | Where |
|---------|-------|
| `logging.getLogger()` integration | Receiver |
| Multiple log levels (debug, info, warn, error) | Receiver |
| `--log-level` CLI filtering | CLI flag |
| Automatic stdout/stderr capture | Adora runtime |
