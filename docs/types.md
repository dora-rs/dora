# Type Annotations

Optional type annotations on dataflow inputs and outputs. Types are never required -- unannotated ports remain fully dynamic. Type checking runs at build time and validate time (no runtime overhead by default).

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
dora validate dataflow.yml

# Fail with non-zero exit code on warnings (for CI)
dora validate --strict-types dataflow.yml

# Type checks also run during build
dora build dataflow.yml --strict-types
```

You can also set `strict_types: true` at the top level of the YAML to enable strict mode without the CLI flag:

```yaml
strict_types: true
nodes:
  # ...
```

## Type URN Format

Type URNs follow the pattern `std/<category>/v<version>/<TypeName>`:

```
std/core/v1/Float32
std/media/v1/Image
std/vision/v1/BoundingBox
```

### Parameterized Types

Some struct types accept parameters to distinguish variants:

```
std/media/v1/AudioFrame[sample_type=f32]
std/media/v1/AudioFrame[sample_type=f32,channels=2]
```

Matching rules:
- Same base + same params -> compatible
- Same base + one side unparameterized -> compatible (wildcard)
- Same base + different param values -> **mismatch**

```yaml
# These are compatible (wildcard):
output_types:
  audio: std/media/v1/AudioFrame[sample_type=f32]
input_types:
  audio: std/media/v1/AudioFrame

# These are a mismatch:
output_types:
  audio: std/media/v1/AudioFrame[sample_type=f32]
input_types:
  audio: std/media/v1/AudioFrame[sample_type=i16]
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
| `Bytes` | LargeBinary | Raw bytes (universal sink -- any type is compatible) |
| `Bool` | Boolean | Boolean |

### `std/math/v1`

| Type | Arrow Type | Fields | Description |
|------|-----------|--------|-------------|
| `Vector3` | Struct | x, y, z (Float64) | 3D vector |
| `Quaternion` | Struct | x, y, z, w (Float64) | Quaternion |
| `Pose` | Struct | position, orientation | 6-DOF pose |
| `Transform` | Struct | translation, rotation | Coordinate transform |

### `std/control/v1`

| Type | Arrow Type | Description |
|------|-----------|-------------|
| `Twist` | Struct | Linear and angular velocity |
| `JointState` | Struct | Joint positions, velocities, efforts |
| `Odometry` | Struct | Pose + Twist in a reference frame |

### `std/media/v1`

| Type | Arrow Type | Parameters | Description |
|------|-----------|------------|-------------|
| `Image` | Struct | `encoding` | Raw image (width, height, encoding, data) |
| `CompressedImage` | LargeBinary | `format` | JPEG/PNG compressed image |
| `PointCloud` | Struct | `point_type` | 3D point cloud |
| `AudioFrame` | Struct | `sample_type` (default: f32) | Audio samples |

### `std/vision/v1`

| Type | Arrow Type | Description |
|------|-----------|-------------|
| `BoundingBox` | Struct | 2D bounding box with confidence and label |
| `Detection` | Struct | Object detection result (list of BoundingBox) |
| `Segmentation` | Struct | Pixel-level segmentation mask |

## Validation Rules

`dora validate` and `dora build` check:

1. **Key existence**: `output_types` keys must appear in `outputs`, `input_types` keys must appear in `inputs`
2. **URN resolution**: All type URNs must exist in the standard or user-defined type library. Typos get "did you mean?" suggestions.
3. **Edge compatibility**: Connected edges must have compatible types (exact match, implicit widening, or user-defined rules)
4. **Timer auto-typing**: Timer inputs (`dora/timer/*`) are automatically typed as `std/core/v1/UInt64`
5. **Type inference**: When only the upstream side annotates a type, it is inferred on the downstream input and reported
6. **Parameterized types**: Parameter mismatches are detected (see above)
7. **Metadata patterns**: `output_metadata` keys and `pattern` shorthands are validated (see below)
8. **Schema compatibility**: Struct types are checked at the field level -- missing fields or wrong field types are flagged

All checks produce warnings (non-fatal by default). Use `--strict-types` to treat warnings as errors for CI pipelines.

```
Type warnings:
  - node "camera": output_types key "framez" not found in outputs list
  - node "detector": unknown type "std/vision/v1/BoundingBx" on output "bbox"
    (did you mean "std/vision/v1/BoundingBox"?)
  - node "detector": type mismatch on input "image": upstream camera/image
    declares "std/core/v1/Bytes", but expected "std/media/v1/Image"

Inferred types:
  inferred std/core/v1/Float64 on processor/reading (from sensor/reading)
```

## Type Compatibility Rules

Beyond exact matching, the type checker supports implicit widening conversions:

| From | To |
|------|-----|
| `UInt8` | `UInt32` |
| `UInt32` | `UInt64` |
| `Int32` | `Int64` |
| `Float32` | `Float64` |
| Any type | `Bytes` (universal sink) |

Widening is transitive up to depth 3 (e.g. `UInt8` -> `UInt32` -> `UInt64` works, but chains of 4+ do not).

### User-Defined Compatibility Rules

Add custom rules in the dataflow YAML:

```yaml
type_rules:
  - from: myproject/SensorV1
    to: myproject/SensorV2

nodes:
  # ...
```

## Metadata Patterns

Nodes that implement communication patterns (services, actions) can declare required metadata keys on their outputs.

### Explicit metadata

```yaml
- id: server
  path: server.py
  outputs:
    - response
  output_metadata:
    response: [request_id]
```

### Pattern shorthand

Use the `pattern` field to auto-imply required metadata keys:

```yaml
- id: server
  path: server.py
  pattern: service-server
  outputs:
    - response
```

| Pattern | Required metadata keys |
|---------|----------------------|
| `service-server` | `request_id` |
| `service-client` | `request_id` |
| `action-server` | `goal_id`, `goal_status` |
| `action-client` | `goal_id` |

## User-Defined Types

Projects can define custom types in a `types/` directory next to the dataflow. The directory structure determines the URN prefix:

```
project/
  dataflow.yml
  types/
    myproject/
      sensors/
        v1.yml    # URN prefix: myproject/sensors/v1
```

Type YAML files use the same format as the standard library:

```yaml
types:
  MySensor:
    arrow: Struct
    description: Custom sensor reading
    fields:
      - name: temperature
        type: Float32
      - name: humidity
        type: Float32
```

This creates the URN `myproject/sensors/v1/MySensor`.

The `std/` prefix is reserved and cannot be used for user types.

User types are loaded automatically by `dora validate` and `dora build` when a `types/` directory exists.

## Runtime Type Checking

In addition to static validation, Dora supports optional runtime type checking on `send_output()`. When enabled, the actual Arrow data type is compared against the declared `output_types` at send time.

Enable via environment variable:

```bash
# Warn on mismatches (log and continue)
DORA_RUNTIME_TYPE_CHECK=warn dora run dataflow.yml

# Error on mismatches (node returns error)
DORA_RUNTIME_TYPE_CHECK=error dora run dataflow.yml
```

Valid values: `1`, `warn`, `true` (warn mode), `error` (error mode). Unset or any other value disables checking (zero overhead).

**Scope:**
- Validates `output_types` on the sender side (`send_output()` calls). `input_types` are checked statically by `dora validate` but not enforced at runtime
- Covers all languages that send Arrow arrays (Rust, Python, C++ Arrow path)
- Raw byte sends (`send_output_bytes`, C nodes) are untyped and skip checking
- Complex types (Struct-based: Image, Vector3, etc.) are skipped -- only primitive types, String, Bytes, and Bool are validated at runtime
- **Pattern messages are skipped.** When a message carries `request_id`, `goal_id`, or `goal_status` in its metadata parameters, runtime type checking is bypassed for that message. Service, action, and streaming patterns legitimately multiplex multiple Arrow schemas through a single output — a service server may reply with different response shapes for different request types — so a single declared Arrow type cannot cover all variants. Non-pattern messages on the same output are still validated normally (dora-rs/adora#150).

The same exemption applies to the receive-side first-message type check: if the first input message carries pattern metadata, the check stays armed and defers to the next non-pattern message. This avoids false positives on inputs that receive a polymorphic first message.

## Graph Visualization

When outputs have type annotations, `dora graph` shows the type on edge labels:

```bash
dora graph dataflow.yml --open
```

Edges display as `output_name [TypeName]` (e.g. `image [Image]`).

## Operators

Operators support the same `output_types`, `input_types`, `output_metadata`, and `pattern` fields:

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
