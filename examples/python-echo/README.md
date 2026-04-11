# Python Echo

Minimal three-node pipeline demonstrating end-to-end data passing and validation in Dora.

## Architecture

```
timer (500ms) --> sender --> data --> echo --> data --> checker
```

## Nodes

**sender** (`sender.py`) — Triggered every 500 ms by the built-in timer. Sends a fixed PyArrow array `[1, 2, 3, 4, 5]` downstream, forwarding the event metadata unchanged.

**echo** (`echo.py`) — Receives the array and re-sends it verbatim (value + metadata). Acts as a transparent relay to verify that data survives a hop between nodes without modification.

**checker** (`checker.py`) — Receives the echoed array and compares it against the expected value `[1, 2, 3, 4, 5]`. Prints `[PASS]` or `[FAIL]` for every message and a final count when the dataflow stops.

## Prerequisites

Install the Python node API (the PyPI package is `dora-rs`, **not** `dora`):

```bash
pip install dora-rs pyarrow
```

> **Note:** The Python import name is `dora` (`from dora import Node`), but the
> PyPI package name is **`dora-rs`**. Running `pip install dora` installs an
> unrelated package and will cause `ImportError: cannot import name 'Node'`.

## Run

```bash
dora run dataflow.yml
```

Expected output:

```
[PASS] #1 data matches: [1, 2, 3, 4, 5]
[PASS] #2 data matches: [1, 2, 3, 4, 5]
...
Total PASS: N
```

## What This Demonstrates

| Feature | Where |
|---------|-------|
| Timer-triggered nodes (`dora/timer/millis/N`) | Sender |
| `pa.array()` data serialization | Sender |
| Metadata forwarding (`event["metadata"]`) | Sender, Echo |
| Transparent relay node | Echo |
| `event["value"].to_pylist()` for reading | Checker |
| Data validation across nodes | Checker |
