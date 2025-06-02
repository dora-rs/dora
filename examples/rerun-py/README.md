# Python Dataflow Example

This examples shows how to create and connect dora to rerun py.

```
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
    path: ./node-hub/dora-rerun-py/dora_rerun_py/main.py
    inputs:
      image: camera/image
      boxes2d: object-detection/bbox

```

## Getting Started

```bash
dora run yolo.yml --uv
```
