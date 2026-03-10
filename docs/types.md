# Type Annotations

Optional type annotations on dataflow inputs and outputs. Types are never required -- unannotated ports remain fully dynamic. Type checking is static only (no runtime overhead).

## Quick Start

```yaml
nodes:
  - id: camera
    path: camera.py
    outputs:
      - image
    output_types:
      image: std/media/v1/Image

  - id: detector
    path: detect.py
    inputs:
      image: camera/image
    input_types:
      image: std/media/v1/Image
    outputs:
      - bbox
    output_types:
      bbox: std/vision/v1/BoundingBox
```

Validate with:

```bash
adora validate dataflow.yml

# Fail with non-zero exit code on warnings (for CI)
adora validate --strict dataflow.yml
```

## Type URN Format

Type URNs follow the pattern `std/<category>/v<version>/<TypeName>`:

```
std/core/v1/Float32
std/media/v1/Image
std/vision/v1/BoundingBox
```

## Standard Type Library

### `std/core/v1`

| Type | Arrow Type | Description |
|------|-----------|-------------|
| `Float32` | Float32 | 32-bit float |
| `Float64` | Float64 | 64-bit float |
| `Int32` | Int32 | 32-bit signed integer |
| `Int64` | Int64 | 64-bit signed integer |
| `UInt8` | UInt8 | 8-bit unsigned integer |
| `UInt32` | UInt32 | 32-bit unsigned integer |
| `UInt64` | UInt64 | 64-bit unsigned integer |
| `String` | Utf8 | UTF-8 string |
| `Bytes` | LargeBinary | Raw bytes |
| `Bool` | Boolean | Boolean |

### `std/math/v1`

| Type | Arrow Type | Description |
|------|-----------|-------------|
| `Vector3` | Struct | 3D vector (x, y, z Float64) |
| `Quaternion` | Struct | Quaternion (x, y, z, w Float64) |
| `Pose` | Struct | 6-DOF pose (position + orientation) |
| `Transform` | Struct | Coordinate transform (translation + rotation) |

### `std/control/v1`

| Type | Arrow Type | Description |
|------|-----------|-------------|
| `Twist` | Struct | Linear and angular velocity |
| `JointState` | Struct | Joint positions, velocities, efforts |
| `Odometry` | Struct | Pose + Twist in a reference frame |

### `std/media/v1`

| Type | Arrow Type | Description |
|------|-----------|-------------|
| `Image` | Struct | Raw image (width, height, encoding, data) |
| `CompressedImage` | LargeBinary | JPEG/PNG compressed image |
| `PointCloud` | Struct | 3D point cloud |
| `AudioFrame` | Struct | Audio samples |

### `std/vision/v1`

| Type | Arrow Type | Description |
|------|-----------|-------------|
| `BoundingBox` | Struct | 2D bounding box with confidence and label |
| `Detection` | Struct | Object detection result (list of BoundingBox) |
| `Segmentation` | Struct | Pixel-level segmentation mask |

## Validation Rules

`adora validate` checks:

1. **Key existence**: `output_types` keys must appear in `outputs`, `input_types` keys must appear in `inputs`
2. **URN resolution**: All type URNs must exist in the standard library
3. **Edge compatibility**: When a connected output and input both have types, they must match

All checks produce warnings (non-fatal by default). Use `--strict` to treat warnings as errors for CI pipelines. Typos get suggestions:

```
Type warnings:
  - node "camera": output_types key "framez" not found in outputs list
  - node "detector": unknown type "std/vision/v1/BoundingBx" on output "bbox"
    (did you mean "std/vision/v1/BoundingBox"?)
  - node "detector": type mismatch on input "image": upstream camera/image
    declares "std/core/v1/Bytes", but expected "std/media/v1/Image"
```

## Runtime Type Checking

In addition to static validation, Adora supports optional runtime type checking on `send_output()`. When enabled, the actual Arrow data type is compared against the declared `output_types` at send time.

Enable via environment variable:

```bash
# Warn on mismatches (log and continue)
ADORA_RUNTIME_TYPE_CHECK=warn adora run dataflow.yml

# Error on mismatches (node returns error)
ADORA_RUNTIME_TYPE_CHECK=error adora run dataflow.yml
```

Valid values: `1`, `warn`, `true` (warn mode), `error` (error mode). Unset or any other value disables checking (zero overhead).

**Scope:**
- Validates `output_types` on the sender side (`send_output()` calls). `input_types` are checked statically by `adora validate` but not enforced at runtime
- Covers all languages that send Arrow arrays (Rust, Python, C++ Arrow path)
- Raw byte sends (`send_output_bytes`, C nodes) are untyped and skip checking
- Complex types (Struct-based: Image, Vector3, etc.) are skipped — only primitive types, String, Bytes, and Bool are validated at runtime

## Graph Visualization

When outputs have type annotations, `adora graph` shows the type on edge labels:

```bash
adora graph dataflow.yml --open
```

Edges display as `output_name [TypeName]` (e.g. `image [Image]`).

## Operators

Operators support the same `output_types` and `input_types` fields:

```yaml
- id: runtime-node
  operators:
    - id: preprocessor
      python: preprocess.py
      inputs:
        raw: sensor/data
      input_types:
        raw: std/core/v1/Bytes
      outputs:
        - processed
      output_types:
        processed: std/media/v1/Image
```
