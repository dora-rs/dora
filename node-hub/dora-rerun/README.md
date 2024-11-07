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
- jointstate: Float32Array

## (Experimental) For plotting 3D URDF

```bash
pip install git+https://github.com/rerun-io/rerun-loader-python-example-urdf.git
```

Make sure to name the dataflow as follows:

```yaml
- id: rerun
  path: dora-rerun
  inputs:
    jointstate_<ENTITY_NAME>: <ENTITY_NAME>/jointstate
  env:
    <ENTITY_NAME>_urdf: /path/to/<ENTITY_NAME>.urdf
    <ENTITY_NAME>_transform: 0 0.3 0
```

> [!IMPORTANT]  
> Make sure that the urdf file name correspond to your dataflow object name otherwise, it will not be able to link to the corresponding entity.

## Configurations

- RERUN_MEMORY_LIMIT: Rerun memory limit
