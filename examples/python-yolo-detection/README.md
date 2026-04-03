# Python YOLO Detection

Real-time object detection pipeline using YOLOv8-nano and OpenCV, with an optional multi-camera variant.

## Architecture

### Single camera (`dataflow.yml`)

```
timer (50ms) --> webcam --> image --> object_detection --> bbox --> plot
                       \--> image ---------------------------------> plot
```

### Multi-camera (`dataflow_multi.yml`)

Two independent single-camera pipelines run in parallel:

```
timer --> webcam_0 --> object_detection_0 --> plot_0  (Camera 0)
timer --> webcam_1 --> object_detection_1 --> plot_1  (Camera 1)
```

## Nodes

**webcam** (`webcam.py`) — Captures 640×480 frames from a USB camera every 50 ms. If the camera fails for more than 10 consecutive reads the node releases and re-opens the device, providing basic hot-plug recovery.

**object_detection** (`object_detection.py`) — Loads `yolov8n.pt` and runs inference on each incoming frame (CPU-only to avoid GPU memory contention between parallel nodes). Outputs a flattened array of shape `(N, 6)` containing `[x1, y1, x2, y2, confidence, class_id]` for each detected object.

**plot** (`plot.py`) — Receives both raw frames and bounding boxes. Draws labelled rectangles on each frame using the cached bbox list and displays the result in an OpenCV window. Press **q** to quit.

**utils** (`utils.py`) — COCO class label list (80 classes) used by the plot node.

## Prerequisites

Download the YOLOv8-nano weights before running:

```bash
pip install adora-rs numpy opencv-python pyarrow ultralytics
python -c "from ultralytics import YOLO; YOLO('yolov8n.pt')"
```

> **Note:** The PyPI package is `adora-rs`, not `adora`. The model weights (`yolov8n.pt`) are downloaded automatically by the `ultralytics` package and are excluded from the repository via `.gitignore`.

## Run

Single camera:

```bash
adora run dataflow.yml
```

Two cameras simultaneously:

```bash
adora run dataflow_multi.yml
```

Set `CAMERA_INDEX` in the dataflow YAML to select a different camera device.

## What This Demonstrates

| Feature | Where |
|---------|-------|
| Computer vision pipeline | All nodes |
| Hot-plug camera recovery | Webcam |
| ML model inference (YOLOv8) | Object detection |
| Async frame/bbox synchronization | Plot |
| Multiple inputs on one node (`image` + `bbox`) | Plot |
| `event["id"]` to distinguish inputs | Plot |
| `restart_policy: on-failure` (Adora-specific) | Detection, Plot |
| Parallel pipelines for multiple cameras | `dataflow_multi.yml` |
| `WINDOW_NAME` env var for multi-window display | Plot (multi) |
