# Typed Dataflow

Demonstrates **type annotations** — optional but powerful for catching
wiring mistakes at build time, before running anything.

## Architecture

```
sensor --> reading (Float64) --> processor --> result (String) --> sink
```

Each edge carries a declared type. `adora validate` checks that the type
leaving one node matches the type expected by the next.

## Nodes

**sensor** (`sensor.py`) — Emits 50 simulated temperature readings drawn
from `N(20.0, 2.0)` as `Float64` Arrow arrays, one every 100 ms.

**processor** (`processor.py`) — Receives each `Float64` reading and
converts it to a human-readable `String` label:
- `> 22.0` → `"HIGH: 22.1"`
- `< 18.0` → `"LOW: 17.2"`
- otherwise → `"OK: 19.8"`

**sink** (`sink.py`) — Prints each label to stdout.

## Validate Types Before Running

```bash
# Check for type mismatches (warnings only)
adora validate dataflow.yml
```

Output when all types match:

```
Validating examples/typed-dataflow/dataflow.yml...
All type annotations OK.
```

For CI, treat any warning as a failure:

```bash
adora validate --strict-types dataflow.yml
```

## See What a Mismatch Looks Like

Change `processor`'s `input_types` to the wrong type and run validate:

```yaml
# dataflow.yml — intentional mismatch
- id: processor
  input_types:
    reading: std/core/v1/Int32    # wrong: sensor outputs Float64
```

```bash
adora validate dataflow.yml
```

You will see a type mismatch warning showing the declared upstream type
versus the expected downstream type. Restore `std/core/v1/Float64` before
running.

## Run

```bash
pip install adora-rs pyarrow
adora run dataflow.yml
```

Expected output (values vary — readings are random):

```
[sink] OK: 21.5
[sink] OK: 20.1
[sink] OK: 21.7
[sink] HIGH: 22.1
[sink] LOW: 16.3
[sink] OK: 19.6
[sink] OK: 18.6
[sink] HIGH: 22.7
[sink] HIGH: 24.3
[sink] LOW: 17.2
...
```

## Visualize the Graph With Type Labels

```bash
adora graph dataflow.yml --open
```

Edges display the type name (e.g. `reading [Float64]`) when annotations
are present.

## What This Demonstrates

| Feature | Where |
|---------|-------|
| `output_types:` on a node | `sensor` in `dataflow.yml` |
| `input_types:` on a node | `processor`, `sink` in `dataflow.yml` |
| Standard type URNs (`std/core/v1/Float64`) | `dataflow.yml` |
| `adora validate` for static checking | CLI |
| `adora validate --strict-types` for CI | CLI |
| Type labels on graph edges | `adora graph --open` |

## Key Concept: Type URNs

Types follow the pattern `std/<category>/v<version>/<TypeName>`:

```
std/core/v1/Float64
std/core/v1/String
std/media/v1/Image
std/vision/v1/BoundingBox
```

Types are **never required** — unannotated ports remain fully dynamic.
Adding annotations is purely opt-in and adds no runtime overhead.
