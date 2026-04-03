# Python Parquet Recorder

Records live camera frames to Parquet files using a two-node pipeline with a ready-signal handshake.

## Architecture

```
recorder --[status: "ready"]--> camera
camera   --[image + metadata]--> recorder --> logs/frames_NNNN.parquet
```

The recorder starts first and sends a `"ready"` status message. The camera waits for this signal before opening the device and sending frames. This handshake prevents lost frames caused by a slow recorder startup.

## Nodes

**recorder** (`recorder.py`) — Signals readiness on startup, then buffers incoming frames in memory. When the buffer reaches `BATCH_SIZE` frames it writes them as a Parquet file under `LOG_DIR`. Each row stores `timestamp_ns`, `frame_id`, `width`, `height`, `encoding`, and the raw pixel `data` as bytes. Any remaining frames are flushed on exit.

**camera** (`camera.py`) — Waits for the recorder's ready signal, then captures frames from a USB camera (`CAMERA_INDEX`) at 640×480 every 50 ms. Each frame is flattened to a 1-D PyArrow array and sent with metadata (`width`, `height`, `encoding`, `frame_id`).

## Prerequisites

```bash
pip install adora-rs numpy opencv-python pyarrow pandas
```

> **Note:** The PyPI package is `adora-rs`, not `adora`.

## Configuration

| Environment variable | Default | Description |
|----------------------|---------|-------------|
| `CAMERA_INDEX` | `0` | Camera device index passed to `cv2.VideoCapture` |
| `LOG_DIR` | `logs` | Directory where Parquet files are written |
| `BATCH_SIZE` | `30` | Number of frames per Parquet file |

## Run

```bash
adora run dataflow.yml
```

Recorded files appear in `logs/`:

```
logs/frames_0000.parquet   # frames 0-29
logs/frames_0001.parquet   # frames 30-59
...
```

Each Parquet file has the schema:

| Column | Type | Description |
|--------|------|-------------|
| `timestamp_ns` | int64 | Host monotonic timestamp (ns) |
| `frame_id` | int | Sequential frame counter |
| `width` | int | Frame width in pixels |
| `height` | int | Frame height in pixels |
| `encoding` | str | Pixel format (`bgr8`) |
| `data` | bytes | Raw pixel data |

## What This Demonstrates

| Feature | Where |
|---------|-------|
| Node startup handshake (ready signal) | Recorder → Camera |
| Metadata alongside data payloads | Camera |
| Batch buffering before disk write | Recorder |
| Parquet serialization with pandas | Recorder |
| `restart_policy: on-failure` (Adora-specific) | Both nodes |
| Environment variable configuration | Both nodes |
