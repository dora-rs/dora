# dora-rerun-py

A Dora node for visualizing data streams using [Rerun](https://rerun.io). Enables real-time rendering of images, bounding boxes, 3D point clouds, URDF models, text logs, and time-series data within Dora workflows.

## Features

- **Image Rendering**: Display incoming camera frames or any image buffers.
- **Bounding Boxes**: Visualize 2D bounding boxes with labels (`xyxy` or `xywh`).
- **Point Clouds**: Render depth data as 3D point clouds, with optional color overlay.
- **URDF Visualization**: Load and animate 3D URDF models based on joint states.
- **Text Logs**: Stream textual logs directly into the Rerun UI.
- **Time Series**: Plot numeric series as scalar timelines.
- **Flexible Configuration**: Adjust memory limits, operating modes, and logging via environment variables.

## Installation

Install in editable mode during development:
```bash
python -m venv .venv
source .venv/bin/activate   # or `.venv\Scripts\activate` on Windows
pip install -e .
```

Or install from PyPI (when released):
```bash
pip install dora-rerun-py
```

## Usage

### As a Dora Node

Define your Dora dataflow YAML and include **dora-rerun-py**:

```yaml
nodes:
  - id: plot
    build: pip install dora-rerun-py
    path: dora-rerun-py
    inputs:
      image: camera/image       # UInt8Array + metadata {width, height, encoding}
      boxes2d: detector/bbox    # StructArray + metadata {format}
      depth: camera/depth       # Float64Array + metadata {width, focal, resolution}
      mask: detector/mask       # Float32 or BooleanArray
      text: logger/text         # StringArray
      series: metrics/series    # Float64Array
      jointstate_robot: robot/jointstate  # Float32Array
    env:
      RERUN_MEMORY_LIMIT: 25%   # Adjust memory usage
      OPERATING_MODE: SPAWN     # SPAWN, CONNECT, or SAVE
```

Then run:
```bash
dora run your_flow.yml --uv
```

### Environment Variables

- `RERUN_MEMORY_LIMIT` (e.g. `25%`): Cap Rerun's memory usage.
- `OPERATING_MODE`: One of `SPAWN`, `CONNECT`, or `SAVE` (default: `SPAWN`).
- `<NAME>_urdf`: Path to a URDF file for 3D visualization.
- `<NAME>_transform`: Initial transform for the URDF model (x y z).
- `README`: Log an initial text document into Rerun.

## Integration Examples

#### Object Detection Pipeline

```yaml
nodes:
  - id: camera
    build: pip install opencv-video-capture
    path: opencv-video-capture
    inputs:
      tick: dora/timer/millis/20
    outputs:
      - image
    env:
      CAPTURE_PATH: 0
      IMAGE_WIDTH: 640
      IMAGE_HEIGHT: 480

  - id: object-detection
    build: pip install dora-yolo
    path: dora-yolo
    inputs:
      image: camera/image
    outputs:
      - bbox

  - id: plot
    build: pip install dora-rerun-py
    path: dora-rerun
    inputs:
      image: camera/image
      boxes2d: object-detection/bbox

```

## System Requirements

- Python ≥ 3.10
- Dora CLI ≥ 0.3.9
- Rerun SDK 0.22.0
- Optional: URDF loader for Python (for 3D models)

## Known Limitations

- API and data formats are experimental and may change.
- URDF loader must be installed separately: `pip install git+https://github.com/rerun-io/rerun-loader-python-example-urdf.git`.
- No transparent colors in current Rerun versions.

## License

Released under the MIT License. See [LICENSE](LICENSE) for details.

