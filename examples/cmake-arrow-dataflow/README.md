# CMake Arrow Dataflow Example

This example shows how to build a Dora C++ node with **CMake** that sends and receives **typed Arrow arrays** (Int32, Float64, String) between nodes.

It demonstrates the Arrow C FFI pattern for exchanging typed data — the same approach used by PyArrow's `_import_from_c` / `_export_to_c`.

## Prerequisites

- [Apache Arrow C++](https://arrow.apache.org/install/) installed:
  - **Ubuntu**: `sudo apt install -y libarrow-dev`
  - **macOS**: `brew install apache-arrow`
- Rust toolchain (for building the dora libraries)
- CMake >= 3.15

## Build

```bash
# From this directory
mkdir build && cd build
cmake .. -DDORA_ROOT_DIR=../../..
cmake --build .
```

## Run

```bash
dora up
dora start ../dataflow.yml
```

You should see both nodes printing typed Arrow arrays (Int32, Float64, String) as they pass data back and forth.

## How it works

The node uses two helper functions that wrap the Arrow C FFI boilerplate:

- **`receive_arrow(event)`** — calls `event_as_arrow_input_with_info()` + `arrow::ImportArray()` to get a `std::shared_ptr<arrow::Array>`
- **`send_arrow(sender, id, array)`** — calls `arrow::ExportArray()` + `send_arrow_output()` to send any Arrow array type

This keeps user code clean while supporting all Arrow types through the standard C Data Interface.
