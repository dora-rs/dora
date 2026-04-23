# Python Multiple Arrays

Demonstrates efficient multi-array messaging -- sending multiple numpy arrays (e.g., images, sensor readings) in a single dora message with near-zero-copy performance.

## Architecture

```
timer (1s) --> sender --> multi_array_msg --> receiver
```

## The Problem

Using `numpy_array.tolist()` to package data is slow because it converts every element into a Python object. For large arrays (images, point clouds), this dominates processing time.

## The Solution

Keep data in binary format using `numpy.ravel()` to flatten arrays, then pass directly to `pyarrow`. On the receiving end, convert back to numpy and reshape.

## Nodes

**sender** (`sender.py`) -- Creates three numpy arrays (simulating two 480x640x3 images and a 1x6 state vector), flattens them with `ravel()`, and packs them into a single Arrow `StructArray`.

**receiver** (`receiver.py`) -- Extracts the binary data, converts back to numpy arrays, and reshapes using hardcoded dimensions (shapes are known by convention between sender and receiver). Prints timing for encode/decode.

## Run

```bash
pip install numpy pyarrow
dora run dataflow.yml
```

Expected output:

```
Sent message with 3 arrays. Encoding time: 0.000345s
Received and decoded. Shape1: (480, 640, 3), Shape2: (480, 640, 3), State: (1, 6). Time: 0.000210s
```

## What This Demonstrates

| Feature | Where |
|---------|-------|
| `numpy.ravel()` for zero-copy flattening | Sender |
| Binary Arrow arrays for large payloads | Both nodes |
| Hardcoded shape reconstruction | Receiver reshapes with known dimensions |
| Sub-millisecond encode/decode | Timing output |
