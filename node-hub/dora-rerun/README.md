# dora-rerun

dora visualization using `rerun`

This nodes is still experimental and format for passing Images, Bounding boxes, and text are probably going to change in the future.

## Changes in v0.24.0

This version introduces significant breaking changes to align with Rerun SDK v0.24.0 and improve the visualization primitive system:

### Major Breaking Changes

1. **Primitive-based Visualization System**
   - **BREAKING**: All inputs now require a `primitive` metadata field to specify the visualization type
   - Previously, visualization type was inferred from the input ID (e.g., "image", "depth", "boxes2d")
   - Now you must explicitly specify: `metadata: { "primitive": "image" }` (or "depth", "boxes2d", etc.)
   - This change allows more flexible naming of inputs and clearer intent

2. **Rerun SDK Upgrade**
   - Updated from Rerun v0.23.3 to v0.24.0
   - Updated Python dependency from `rerun_sdk>=0.23.1` to `rerun_sdk>=0.24.0`

3. **New 3D Boxes Support**
   - Added comprehensive 3D bounding box visualization with multiple format support
   - Supports three formats: "center_half_size" (default), "center_size", and "min_max"
   - Configurable rendering: wireframe (default) or solid fill
   - Support for per-box colors and labels

4. **Enhanced Depth Visualization**
   - Depth data now supports pinhole camera setup for proper 3D reconstruction
   - Requires Float32Array format (previously supported Float64 and UInt16)
   - New metadata fields for camera configuration:
     - `camera_position`: [x, y, z] position
     - `camera_orientation`: [x, y, z, w] quaternion
     - `focal`: [fx, fy] focal lengths
     - `principal_point`: [cx, cy] principal point (optional)
   - Without camera metadata, depth is logged but 3D reconstruction is skipped

5. **Removed Features**
   - Removed series/time-series visualization support
   - Removed legacy camera pitch configuration via CAMERA_PITCH environment variable
   - Removed automatic depth-to-3D point cloud conversion without proper camera parameters

### Migration Guide

#### Before (old system):
```yaml
nodes:
  - id: rerun
    inputs:
      image: camera/image         # Type inferred from "image" in ID
      depth: sensor/depth         # Type inferred from "depth" in ID
      boxes2d: detector/boxes2d   # Type inferred from "boxes2d" in ID
```

#### After (new system):
```yaml
nodes:
  - id: rerun
    inputs:
      camera_feed: camera/image
      depth_sensor: sensor/depth
      detections: detector/boxes2d
    # Visualization types must be specified in the sender's metadata:
    # camera/image must send: metadata { "primitive": "image" }
    # sensor/depth must send: metadata { "primitive": "depth" }
    # detector/boxes2d must send: metadata { "primitive": "boxes2d" }
```

### Migration Status

**Successfully tested examples:**
- ✅ examples/rerun-viewer/dataflow.yml - Basic camera visualization
- ✅ examples/camera/dataflow_rerun.yml - Camera with rerun
- ✅ examples/python-dataflow/dataflow.yml - Camera + YOLO object detection
- ✅ examples/python-multi-env/dataflow.yml - Multi-environment setup

**Updated but NOT tested examples:**
- 🔧 examples/keyboard/dataflow.yml - Updated dora-keyboard to send primitive metadata (requires Linux/X11 for testing)
- 🔧 examples/translation/* - Updated dora-distil-whisper and dora-argotranslate to send primitive metadata
- 🔧 examples/reachy2-remote/dataflow_reachy.yml - Updated multiple nodes (dora-reachy2, dora-qwen2-5-vl, dora-sam2, parse_bbox.py, parse_whisper.py)
- 🔧 examples/lebai/graphs/dataflow_full.yml - Updated dora-qwenvl, llama-factory-recorder, key_interpolation.py
- 🔧 examples/av1-encoding/* - Updated dora-dav1d and dora-rav1e to send primitive metadata

Key changes made:
1. Added `-e` flag to local package installs in dataflows for development
2. Updated node packages to include `"primitive"` metadata:
   - opencv-video-capture: adds `"primitive": "image"`
   - dora-yolo: adds `"primitive": "boxes2d"`
   - dora-keyboard: adds `"primitive": "text"`
   - dora-distil-whisper: adds `"primitive": "text"`
   - dora-argotranslate: adds `"primitive": "text"`
   - dora-sam2: adds `"primitive": "masks"`
   - dora-qwen2-5-vl: adds `"primitive": "text"`
   - dora-qwenvl: adds `"primitive": "text"`
   - dora-reachy2/camera.py: adds appropriate primitives
   - dora-dav1d: adds `"primitive": "image"`
   - dora-rav1e: adds `"primitive": "image"`

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

## Supported Visualization Primitives

All inputs require a `"primitive"` field in the metadata to specify the visualization type:

### 1. image
- **Data**: UInt8Array
- **Required metadata**: `{ "primitive": "image", "width": int, "height": int, "encoding": str }`
- **Supported encodings**: "bgr8", "rgb8", "jpeg", "png", "avif"

### 2. depth  
- **Data**: Float32Array
- **Required metadata**: `{ "primitive": "depth", "width": int, "height": int }`
- **Optional metadata for 3D reconstruction**:
  - `"camera_position"`: [x, y, z] position
  - `"camera_orientation"`: [x, y, z, w] quaternion
  - `"focal"`: [fx, fy] focal lengths
  - `"principal_point"`: [cx, cy] principal point

### 3. text
- **Data**: StringArray
- **Required metadata**: `{ "primitive": "text" }`

### 4. boxes2d
- **Data**: StructArray or Float32Array
- **Required metadata**: `{ "primitive": "boxes2d", "format": str }`
- **Formats**: "xyxy" (default), "xywh"

### 5. boxes3d
- **Data**: Float32Array or StructArray
- **Required metadata**: `{ "primitive": "boxes3d" }`
- **Optional metadata**:
  - `"format"`: "center_half_size" (default), "center_size", "min_max"
  - `"solid"`: bool (default false for wireframe)
  - `"color"`: [r, g, b] RGB values 0-255

### 6. masks
- **Data**: UInt8Array
- **Required metadata**: `{ "primitive": "masks", "width": int, "height": int }`

### 7. jointstate
- **Data**: Float32Array
- **Required metadata**: `{ "primitive": "jointstate" }`
- **Note**: Requires URDF configuration (see below)

### 8. pose
- **Data**: Float32Array (7 values: [x, y, z, qx, qy, qz, qw])
- **Required metadata**: `{ "primitive": "pose" }`

### 9. series
- **Data**: Float32Array
- **Required metadata**: `{ "primitive": "series" }`
- **Note**: Currently logs only the first value as a scalar

### 10. points3d
- **Data**: Float32Array (xyz triplets)
- **Required metadata**: `{ "primitive": "points3d" }`
- **Optional metadata**:
  - `"color"`: [r, g, b] RGB values 0-255
  - `"radii"`: list of float radius values

### 11. points2d
- **Data**: Float32Array (xy pairs)
- **Required metadata**: `{ "primitive": "points2d" }`

### 12. lines3d
- **Data**: Float32Array (xyz triplets defining line segments)
- **Required metadata**: `{ "primitive": "lines3d" }`
- **Optional metadata**:
  - `"color"`: [r, g, b] RGB values 0-255
  - `"radius"`: float line thickness

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
