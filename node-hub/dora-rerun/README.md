# dora-rerun

dora visualization using `rerun`

This nodes is still experimental and format for passing Images, Bounding boxes, and text are probably going to change in the future.

## Getting Started

```bash
pip install dora-rerun
```

## Adding to existing graph:

```yaml
- id: plot
  build: pip install dora-rerun
  path: dora-rerun
  inputs:
    image:
      source: camera/image
      queue_size: 1
    text_qwenvl: dora-qwenvl/text
    text_whisper: dora-distil-whisper/text
  env:
    IMAGE_WIDTH: 640
    IMAGE_HEIGHT: 480
    README: |
      # Visualization
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

> [!WARNING]
> Please make sure to review the following gotchas:
>
> - Filename included in URDF are going to be relative to your dataflow working directory instead of the URDF path: https://github.com/rerun-io/rerun-loader-python-example-urdf/issues/13
> - URDF loader is not on pip and so you need to install it yourself https://github.com/rerun-io/rerun-loader-python-example-urdf/issues/12
> - There is no warning if a file is not logged properly. https://github.com/rerun-io/rerun-loader-python-example-urdf/pull/14
> - There is no transparent color in rerun. https://github.com/rerun-io/rerun/issues/1611

## Configurations

- RERUN_MEMORY_LIMIT: Rerun memory limit

## Reference documentation

- dora-rerun
  - github: https://github.com/dora-rs/dora/blob/main/node-hub/dora-rerun
  - website: http://dora-rs.ai/docs/nodes/rerun
- rerun
  - github: https://github.com/rerun-io/rerun
  - website: https://rerun.io

## Examples

- speech to text
  - github: https://github.com/dora-rs/dora/blob/main/examples/speech-to-text
  - website: https://dora-rs.ai/docs/examples/stt
- vision language model
  - github: https://github.com/dora-rs/dora/blob/main/examples/vlm
  - website: https://dora-rs.ai/docs/examples/vlm

## License

The code and model weights are released under the MIT License.
