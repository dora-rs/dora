# Python Zero-Copy Send

Demonstrates `node.send_output_raw(output_id, length, metadata=)` — a Python API that exposes a pre-allocated output buffer via Python's buffer protocol so callers can write into dora's send memory directly (no copy from Python into dora's send path).

## Why this exists

`node.send_output(...)` with a bytes/Arrow payload performs **one copy** to move the data into dora's shared-memory send buffer. For typical Python ML workloads (camera frames, point clouds, embeddings) sized in the MBs at tens of Hz, this copy is a real cost — both CPU and memory bandwidth.

`send_output_raw` returns a `SampleHandler` that owns the pre-allocated send buffer. Fill it via memoryview / numpy / struct — no intermediate copy. Then call `.send()`, or use the recommended context-manager form which sends on `__exit__`.

## Requirements

- Python >= 3.11 (buffer-protocol slots became part of the stable C API in 3.11)
- numpy (for the example only; the underlying API works with raw `memoryview`)

## Run

```bash
dora run dataflow.yml --uv
```

Expected output: 10 tick-driven iterations, each sending one 64 KiB `large_output` and one 10 000-pixel `rgb_image`. The receiver prints a one-line summary per message.

## Two usage patterns

### Recommended: context manager (sends on `__exit__`)

```python
with node.send_output_raw("frame", height * width * 3, metadata={"width": width}) as buf:
    np.asarray(buf, dtype=np.uint8).reshape(height, width, 3)[:] = frame
# data is sent automatically when the `with` block exits
```

### Manual (when send timing is conditional)

```python
sample = node.send_output_raw("frame", height * width * 3)
mv = sample.as_memoryview()
arr = np.asarray(mv, dtype=np.uint8).reshape(height, width, 3)
arr[:] = frame
del arr        # drop derived numpy view first
mv.release()   # release the memoryview itself
sample.send()  # ship it
```

## Safety contract

The `SampleHandler` enforces a small protocol so the zero-copy path stays sound:

- **Write-only-once.** Calling `.send()` twice on the same handler is an error.
- **No view past `send()`.** Once `.send()` runs, any subsequent attempt to acquire a buffer view raises `BufferError`.
- **No `send()` while views are open.** If you have an outstanding `memoryview` (or a derived numpy array that hasn't been GC'd), `.send()` errors with a clear message. Release the views first.

The context-manager form handles release automatically (`__exit__` releases the cached memoryview before calling send). The manual form is the one where you need to be careful: `del arr` on any numpy view, then `mv.release()`, before `sample.send()`.

## Performance

`send_output_raw` removes one copy in the Python-side send path. For a 1080p RGB frame (~6 MiB) at 30 fps this is ~180 MB/s of CPU and memory-bandwidth savings, plus 50–200 µs of latency depending on cache state.

If the receiver is co-located (same daemon), the full data path becomes: Python writes → dora SHM region → receiver reads same SHM region — no copies anywhere. Cross-daemon receivers still incur a network serialization step on the receive side; that's a separate concern outside the scope of this API.

## Files

| File | Purpose |
|---|---|
| `dataflow.yml` | Sender + receiver, timer-driven (10 Hz) |
| `node.py` | Sender — both usage patterns shown |
| `receiver.py` | Receiver — prints one line per received message |
