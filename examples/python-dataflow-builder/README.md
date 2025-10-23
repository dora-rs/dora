# Python Dataflow Builder - Simple Example

This example demonstrates how to use Dora's Python API to build a simple computer vision dataflow that performs real-time object detection using YOLO.

## Overview

The `simple_example.py` script creates a three-node dataflow:

1. **Camera Node** - Captures video frames from the default camera
2. **Object Detection Node** - Performs YOLO-based object detection on the frames
3. **Plot Node** - Visualizes the detected objects by drawing bounding boxes on the frames

## What it does

The dataflow:

- Captures 640x480 video frames from camera at 50 FPS (every 20ms)
- Runs YOLO object detection on each frame
- Displays the results with bounding boxes overlaid on the original video

## Dependencies

The script automatically installs the following packages:

- `opencv-video-capture` - For camera input
- `dora-yolo` - For YOLO object detection
- `opencv-plot` - For visualization

## Key API Methods

### `add_input(name, source)`

Defines an input port for a node. The input can come from:

- **Timer source**: `"dora/timer/millis/20"` creates a timer that ticks every 20ms
- **Another node's output**: Use the output object returned by `add_output()`

Examples from the code:

```python
# Timer input - triggers camera capture every 20ms
camera.add_input("tick", "dora/timer/millis/20")

# Node-to-node connection - object detection receives images from camera
object_detection.add_input("image", camera_image)

# Multiple inputs - plot receives both images and bounding boxes
plot.add_input("image", camera_image)
plot.add_input("boxes2d", bbox_output)
```

### `add_output(name)`

Defines an output port for a node and returns an output object that can be used as input for other nodes.

Examples from the code:

```python
# Camera outputs images
camera_image = camera.add_output("image")

# Object detection outputs bounding boxes
bbox_output = object_detection.add_output("bbox")
```

The returned output objects (`camera_image`, `bbox_output`) are then used to connect nodes together by passing them to `add_input()` on other nodes.

## Usage

Run the example:

```bash
python simple_example.py
```

To skip the build (useful for testing YAML generation only):

```bash
NO_BUILD=1 python simple_example.py
```

## Generated Files

- `dataflow.yml` - The generated dataflow configuration file that can be run with `dora run`
