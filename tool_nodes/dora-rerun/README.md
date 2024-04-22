# dora-rerun

dora visualization using `rerun`

This nodes is still experimental and format for passing Images, Bounding boxes, and text are probably going to change in the future.

## Getting Started

```bash
cargo install --force rerun-cli@0.15.1

## To install this package
git clone git@github.com:dora-rs/dora.git
cargo install --git https://github.com/dora-rs/dora dora-rerun
```

## Adding to existing graph:

```yaml
- id: rerun
  custom:
    source: dora-rerun
    inputs:
      image: webcam/image
      text: webcam/text
      boxes2d: object_detection/bbox
    envs:
      IMAGE_WIDTH: 960
      IMAGE_HEIGHT: 540
      IMAGE_DEPTH: 3
      RERUN_MEMORY_LIMIT: 25%
```

## Configurations

- IMAGE_WIDTH: Image width in pixels
- IMAGE_HEIGHT: Image height in heights
- IMAGE_DEPTH: Image depth
- RERUN_MEMORY_LIMIT: Rerun memory limit
