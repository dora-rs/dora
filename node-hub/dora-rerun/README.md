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
      RERUN_MEMORY_LIMIT: 25%
```

## Input definition

- image: UInt8Array + metadata { "width": int, "height": int, "encoding": str }
- boxes2D: StructArray + metadata { "format": str }
- text: StringArray

## Configurations

- RERUN_MEMORY_LIMIT: Rerun memory limit
