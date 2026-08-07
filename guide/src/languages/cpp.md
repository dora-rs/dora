# C++ API Reference

Dora provides C++ bindings for both standalone nodes and in-process operators via [CXX](https://cxx.rs/) (Rust-C++ interop). The CXX bridge generates type-safe C++ headers from Rust definitions -- no raw FFI or manual `extern "C"` declarations are needed.

Two crates provide the C++ surface:

| Crate | Library | Use case |
|-------|---------|----------|
| `dora-node-api-cxx` | `libdora_node_api_cxx.a` | Standalone node executable |
| `dora-operator-api-cxx` | `libdora_operator_api_cxx.a` | Shared-library operator loaded by the runtime |

Generated headers: `dora-node-api.h` and `dora-operator-api.h`.

---

## Node API (`dora-node-api-cxx`)

### Initialization

```cpp
#include "dora-node-api.h"

// Initialize a node from environment variables set by the Dora daemon.
// Returns an DoraNode struct containing the event stream and output sender.
// Throws on failure.
DoraNode init_dora_node();
```

### DoraNode

Returned by `init_dora_node()`. Owns the event stream and the output sender for the lifetime of the node.

```cpp
struct DoraNode {
    rust::Box<Events>        events;       // event stream (blocking receiver)
    rust::Box<OutputSender>  send_output;  // output sender
};
```

### Events

Opaque Rust type exposed to C++. Provides blocking iteration over the node's incoming events.

```cpp
// Member function -- call on the boxed object directly.
rust::Box<DoraEvent> Events::next();

// Free function form -- equivalent to events->next().
rust::Box<DoraEvent> next_event(rust::Box<Events>& events);
```

Both forms block until the next event arrives and return an owned `DoraEvent`.

### DoraEvent

Opaque Rust type. Inspect its kind with `event_type()`, then downcast with `event_as_input()` or `event_as_arrow_input()`.

```cpp
// Determine the event kind.
DoraEventType event_type(const rust::Box<DoraEvent>& event);

// Downcast to a raw-byte input. Throws if the event is not Input.
DoraInput event_as_input(rust::Box<DoraEvent> event);

// Downcast to an Arrow FFI input (writes Arrow C Data Interface structs).
// out_array and out_schema must point to valid ArrowArray / ArrowSchema structs.
// Returns DoraResult with empty error on success.
DoraResult event_as_arrow_input(
    rust::Box<DoraEvent> event,
    uint8_t* out_array,
    uint8_t* out_schema);

// Same as above, but also returns the input ID and metadata.
ArrowInputInfo event_as_arrow_input_with_info(
    rust::Box<DoraEvent> event,
    uint8_t* out_array,
    uint8_t* out_schema);
```

### DoraEventType

```cpp
enum class DoraEventType : uint8_t {
    Stop,             // graceful shutdown requested
    Input,            // new data arrived on an input
    InputClosed,      // a single input was closed
    Error,            // an error occurred
    Unknown,          // unrecognized event variant
    AllInputsClosed,  // all inputs closed (stream ended)
};
```

### DoraInput

Returned by `event_as_input()`. Contains raw bytes.

```cpp
struct DoraInput {
    rust::String     id;    // input identifier (e.g. "tick", "image")
    rust::Vec<uint8_t> data;  // raw payload bytes
};
```

### ArrowInputInfo

Returned by `event_as_arrow_input_with_info()`. Contains the input ID, metadata, and an error string.

```cpp
struct ArrowInputInfo {
    rust::String       id;        // input identifier
    rust::Box<Metadata> metadata; // attached metadata
    rust::String       error;     // empty on success
};
```

### DoraResult

Returned by output-sending functions. Check the `error` field -- empty means success.

```cpp
struct DoraResult {
    rust::String error;  // empty string on success
};
```

### OutputSender

Opaque Rust type. All methods take `rust::Box<OutputSender>&` as the first argument (the sender from `DoraNode::send_output`).

#### send_output

Send raw bytes on a named output.

```cpp
DoraResult send_output(
    rust::Box<OutputSender>& sender,
    rust::String id,
    rust::Slice<const uint8_t> data);
```

#### send_output_with_metadata

Send raw bytes with attached metadata.

```cpp
DoraResult send_output_with_metadata(
    rust::Box<OutputSender>& sender,
    rust::String id,
    rust::Slice<const uint8_t> data,
    rust::Box<Metadata> metadata);
```

#### send_arrow_output

Send an Arrow array via the C Data Interface. The pointers must reference valid `ArrowArray` and `ArrowSchema` structs. Ownership of the Arrow data transfers to Rust on success.

```cpp
DoraResult send_arrow_output(
    rust::Box<OutputSender>& sender,
    rust::String id,
    uint8_t* array_ptr,
    uint8_t* schema_ptr);

// Overload with metadata (same C++ name via cxx_name attribute).
DoraResult send_arrow_output(
    rust::Box<OutputSender>& sender,
    rust::String id,
    uint8_t* array_ptr,
    uint8_t* schema_ptr,
    rust::Box<Metadata> metadata);
```

#### log_message

Send a log message through the Dora logging system.

```cpp
DoraResult log_message(
    const rust::Box<OutputSender>& sender,
    rust::String level,    // e.g. "info", "warn", "error"
    rust::String message);
```

### Metadata

Opaque Rust type for attaching typed key-value pairs to outputs.

#### Construction

```cpp
rust::Box<Metadata> new_metadata();
```

#### Reading

```cpp
uint64_t     Metadata::timestamp() const;

bool         Metadata::get_bool(const rust::Str key) const;        // throws on missing/wrong type
int64_t      Metadata::get_int(const rust::Str key) const;
double       Metadata::get_float(const rust::Str key) const;
rust::String Metadata::get_str(const rust::Str key) const;

rust::Vec<int64_t>      Metadata::get_list_int(const rust::Str key) const;
rust::Vec<double>       Metadata::get_list_float(const rust::Str key) const;
rust::Vec<rust::String> Metadata::get_list_string(const rust::Str key) const;

int64_t      Metadata::get_timestamp(const rust::Str key) const;   // nanoseconds since epoch
rust::String Metadata::get_json(const rust::Str key) const;        // single value as JSON string
```

#### Writing

All setters throw on failure.

```cpp
void Metadata::set_bool(const rust::Str key, bool value);
void Metadata::set_int(const rust::Str key, int64_t value);
void Metadata::set_float(const rust::Str key, double value);
void Metadata::set_string(const rust::Str key, rust::String value);

void Metadata::set_list_int(const rust::Str key, rust::Vec<int64_t> value);
void Metadata::set_list_float(const rust::Str key, rust::Vec<double> value);
void Metadata::set_list_string(const rust::Str key, rust::Vec<rust::String> value);

void Metadata::set_timestamp(const rust::Str key, int64_t nanos);  // nanoseconds since epoch
```

#### Introspection

```cpp
MetadataValueType Metadata::type(const rust::Str key) const;  // throws if key missing
rust::String      Metadata::to_json() const;                   // full metadata as JSON
rust::Vec<rust::String> Metadata::list_keys() const;
```

### MetadataValueType

```cpp
enum class MetadataValueType : uint8_t {
    Bool,
    Integer,
    Float,
    String,
    ListInt,
    ListFloat,
    ListString,
    Timestamp,
};
```

### Service, Action, and Streaming Patterns

C++ nodes can implement [communication patterns](patterns.md) using the metadata API. The well-known metadata keys are:

| Key | Description |
|-----|-------------|
| `"request_id"` | Service request/response correlation (UUID v7) |
| `"goal_id"` | Action goal identification (UUID v7) |
| `"goal_status"` | Action result status: `"succeeded"`, `"aborted"`, or `"canceled"` |
| `"session_id"` | Streaming session identifier |
| `"segment_id"` | Streaming segment within a session (integer) |
| `"seq"` | Streaming chunk sequence number (integer) |
| `"fin"` | Last chunk of a streaming segment (bool) |
| `"flush"` | Discard older queued messages on input (bool) |

```cpp
// Service server: pass through request_id from input metadata
auto input_metadata = event_as_arrow_input_with_info(event);
send_output_with_metadata(sender, "response", result, std::move(input_metadata.metadata));

// Action server: set goal_id and goal_status on result
auto meta = new_metadata();
meta->set_string("goal_id", goal_id);
meta->set_string("goal_status", "succeeded");
send_output_with_metadata(sender, "result", result_data, std::move(meta));
```

### CombinedEvents (ROS2 integration)

When using the optional `ros2-bridge` feature, node events and ROS2 subscription events can be merged into a single stream.

```cpp
// Convert Dora events into a combined stream.
CombinedEvents dora_events_into_combined(rust::Box<Events> events);

// Create an empty combined stream (for ROS2-only nodes).
CombinedEvents empty_combined_events();
```

#### CombinedEvents struct

```cpp
struct CombinedEvents {
    rust::Box<MergedEvents> events;

    CombinedEvent next();  // blocking -- returns the next merged event
};
```

#### CombinedEvent struct

```cpp
struct CombinedEvent {
    rust::Box<MergedDoraEvent> event;

    bool is_dora() const;  // true if this is a standard Dora event
};

// Downcast a combined event back to an DoraEvent. Throws if not an Dora event.
rust::Box<DoraEvent> downcast_dora(CombinedEvent event);
```

ROS2 subscriptions add their own events to the merged stream. Use `subscription->matches(event)` and `subscription->downcast(event)` to handle ROS2-specific events (see the [ROS2 Bridge docs](ros2-bridge.md)).

---

## Operator API (`dora-operator-api-cxx`)

Operators are shared libraries loaded by the Dora runtime. The C++ side implements four functions that the CXX bridge calls into.

> **Breaking change vs. earlier dora releases:** the operator API used to require only `new_operator` + `on_input`; `Event::InputClosed` and `Event::Stop` were silently dropped on the C++ side. As of this release the bridge calls `on_input_closed` and `on_stop` instead, so existing C++ operators must add these two functions (a no-op stub returning `{ rust::String(), false }` is sufficient to restore the pre-change behavior). See [#1849](https://github.com/dora-rs/dora/pull/1849) for the rationale.

### Required C++ interface

You must provide a header `operator.h` and an implementation file. The header declares an `Operator` class and four free functions:

```cpp
// operator.h
#pragma once
#include <memory>
#include "dora-operator-api.h"

class Operator {
public:
    Operator();
    // Add any state your operator needs.
};

std::unique_ptr<Operator> new_operator();

DoraOnInputResult on_input(
    Operator& op,
    rust::Str id,
    rust::Slice<const uint8_t> data,
    OutputSender& output_sender);

DoraOnInputResult on_input_closed(Operator& op, rust::Str id, OutputSender& output_sender);
DoraOnInputResult on_stop(Operator& op, OutputSender& output_sender);
```

- `new_operator()` -- called once at startup; returns the operator instance.
- `on_input()` -- called for every input event; process data and optionally send outputs.
- `on_input_closed()` -- called when an upstream input stream closes (the daemon delivers `Event::InputClosed { id }`). Operator can log, flush per-input state, or `send_output(output_sender, ...)` to emit a final/status message in response. Set `result.stop = true` to request shutdown.
- `on_stop()` -- called on graceful shutdown (the daemon delivers `Event::Stop`). Operator can drain output queues, persist final state, or `send_output(output_sender, ...)` to flush buffered data before returning.

Default implementations that simply log + return success are sufficient for operators that don't need to react to these events. The `output_sender` argument is provided so that operators which want to emit final/status outputs on close or stop can do so symmetrically to `on_input`. See `examples/c++-dataflow/operator-rust-api/operator.cc` for a minimal reference.

### OutputSender (operator)

Available inside `on_input()`. Sends data on a named output.

```cpp
DoraSendOutputResult send_output(
    OutputSender& sender,
    rust::Str id,
    rust::Slice<const uint8_t> data);
```

### Result types

```cpp
struct DoraOnInputResult {
    rust::String error;  // empty on success
    bool         stop;   // true to request graceful shutdown
};

struct DoraSendOutputResult {
    rust::String error;  // empty on success
};
```

---

## Quick Start: Node Example

A minimal node that receives timer ticks and sends a counter.

```cpp
#include "dora-node-api.h"
#include <iostream>
#include <vector>

int main() {
    auto dora_node = init_dora_node();
    unsigned char counter = 0;

    for (;;) {
        auto event = next_event(dora_node.events);
        auto ty = event_type(event);

        if (ty == DoraEventType::AllInputsClosed) {
            break;
        }
        if (ty == DoraEventType::Stop) {
            break;
        }
        if (ty == DoraEventType::Input) {
            auto input = event_as_input(std::move(event));
            counter += 1;

            std::cout << "Input: " << std::string(input.id)
                      << " counter=" << (int)counter << std::endl;

            std::vector<unsigned char> out{counter};
            rust::Slice<const uint8_t> slice{out.data(), out.size()};
            auto result = send_output(dora_node.send_output, "counter", slice);
            if (!result.error.empty()) {
                std::cerr << "Send error: " << std::string(result.error) << std::endl;
                return 1;
            }
        }
    }
    return 0;
}
```

Dataflow YAML:

```yaml
nodes:
  - id: cxx-node
    path: build/my_node
    inputs:
      tick: dora/timer/millis/300
    outputs:
      - counter
```

---

## Quick Start: Arrow Node Example

A node that receives and sends Arrow arrays via the C Data Interface, with metadata.

```cpp
#include "dora-node-api.h"
#include <arrow/api.h>
#include <arrow/c/bridge.h>
#include <iostream>

int main() {
    auto dora_node = init_dora_node();

    for (int i = 0; i < 10; i++) {
        auto event = dora_node.events->next();
        auto ty = event_type(event);

        if (ty == DoraEventType::AllInputsClosed || ty == DoraEventType::Stop) {
            break;
        }
        if (ty == DoraEventType::Input) {
            // Receive Arrow input with metadata
            struct ArrowArray c_array;
            struct ArrowSchema c_schema;
            auto info = event_as_arrow_input_with_info(
                std::move(event),
                reinterpret_cast<uint8_t*>(&c_array),
                reinterpret_cast<uint8_t*>(&c_schema));

            if (!info.error.empty()) {
                std::cerr << std::string(info.error) << std::endl;
                continue;
            }

            std::cout << "Input: " << std::string(info.id)
                      << " ts=" << info.metadata->timestamp() << std::endl;

            auto imported = arrow::ImportArray(&c_array, &c_schema);
            auto array = imported.ValueOrDie();
            std::cout << "Arrow: " << array->ToString() << std::endl;

            // Build an output Arrow array
            arrow::Int32Builder builder;
            builder.Append(i * 10);
            std::shared_ptr<arrow::Array> out_array;
            builder.Finish(&out_array);

            // Export and send with metadata
            struct ArrowArray out_c_array;
            struct ArrowSchema out_c_schema;
            arrow::ExportArray(*out_array, &out_c_array, &out_c_schema);

            auto meta = new_metadata();
            meta->set_string("source", "cpp-arrow-node");
            meta->set_int("iteration", i);

            auto result = send_arrow_output(
                dora_node.send_output, "counter",
                reinterpret_cast<uint8_t*>(&out_c_array),
                reinterpret_cast<uint8_t*>(&out_c_schema),
                std::move(meta));

            if (!result.error.empty()) {
                std::cerr << "Send error: " << std::string(result.error) << std::endl;
            }
        }
    }
    return 0;
}
```

---

## Quick Start: Operator Example

A minimal operator shared library.

```cpp
// operator.cc
#include "operator.h"
#include <iostream>
#include <vector>

Operator::Operator() {}

std::unique_ptr<Operator> new_operator() {
    return std::make_unique<Operator>();
}

DoraOnInputResult on_input(
    Operator& op,
    rust::Str id,
    rust::Slice<const uint8_t> data,
    OutputSender& output_sender)
{
    op.counter += 1;

    std::vector<unsigned char> out{op.counter};
    rust::Slice<const uint8_t> slice{out.data(), out.size()};
    auto send_result = send_output(output_sender, rust::Str("status"), slice);

    return DoraOnInputResult{send_result.error, false};
}
```

Dataflow YAML:

```yaml
nodes:
  - id: runtime-node
    operators:
      - id: my-operator
        shared-library: build/my_operator
        inputs:
          data: some-node/output
        outputs:
          - status
```

---

## Build Integration (CMake)

The recommended build approach uses CMake with the `DoraTargets.cmake` helper (see `examples/cmake-dataflow/`).

### Project structure

```
my-project/
  CMakeLists.txt
  DoraTargets.cmake       # copied from examples/cmake-dataflow/
  node/main.cc
  operator/operator.h
  operator/operator.cc
  dataflow.yml
```

### CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.21)
project(my-dataflow LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS "-fPIC")

include(DoraTargets.cmake)
link_directories(${dora_link_dirs})

# Standalone node (executable)
add_executable(my_node node/main.cc ${node_bridge})
add_dependencies(my_node Dora_cxx)
target_include_directories(my_node PRIVATE ${dora_cxx_include_dir})
target_link_libraries(my_node dora_node_api_cxx)

# Operator (shared library)
add_library(my_operator SHARED
    operator/operator.cc ${operator_bridge})
add_dependencies(my_operator Dora_cxx)
target_include_directories(my_operator PRIVATE
    ${dora_cxx_include_dir} ${dora_c_include_dir}
    ${CMAKE_CURRENT_SOURCE_DIR}/operator)
target_link_libraries(my_operator dora_operator_api_cxx)

install(TARGETS my_node DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/bin)
install(TARGETS my_operator DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/lib)
```

### What DoraTargets.cmake provides

| Variable | Description |
|----------|-------------|
| `dora_cxx_include_dir` | Path to generated CXX headers (`dora-node-api.h`, `dora-operator-api.h`) |
| `dora_c_include_dir` | Path to C API headers (for mixed C/C++ projects) |
| `dora_link_dirs` | Library search path for `libdora_node_api_cxx.a` / `libdora_operator_api_cxx.a` |
| `node_bridge` | Generated CXX bridge source file for nodes (`node_bridge.cc`) |
| `operator_bridge` | Generated CXX bridge source file for operators (`operator_bridge.cc`) |
| `Dora_cxx` | CMake target dependency that builds the CXX crates |

### Build steps

```bash
# Option A: Build against local Dora source
mkdir build && cd build
cmake .. -DDORA_ROOT_DIR=/path/to/dora
cmake --build .

# Option B: Build against Dora from GitHub (cloned automatically)
mkdir build && cd build
cmake ..
cmake --build .
```

### Requirements

- C++20 compiler
- Rust toolchain (for building the Dora static libraries via Cargo)
- CMake 3.21+
- For Arrow integration: Apache Arrow C++ library

---

## CXX Bridge Notes

- All Rust opaque types (`Events`, `OutputSender`, `DoraEvent`, `Metadata`, `MergedEvents`, `MergedDoraEvent`) are accessed through `rust::Box<T>`.
- `rust::String`, `rust::Vec<T>`, and `rust::Slice<const T>` are CXX bridge types that interoperate with their C++ standard library counterparts. See the [CXX type reference](https://cxx.rs/binding/box.html).
- Functions that return `Result<T>` in Rust throw C++ exceptions on the error path.
- Arrow FFI functions (`event_as_arrow_input`, `send_arrow_output`) are `unsafe` on the Rust side. The caller must pass valid pointers to `ArrowArray` / `ArrowSchema` structs cast to `uint8_t*`.
- The node library is a static archive (`staticlib`). Link it into your executable with `-ldora_node_api_cxx`.
- The operator library is also a static archive. Link it into your shared library with `-ldora_operator_api_cxx`.

---

## Memory Pools

A memory pool is a named shared-memory segment that several nodes address directly, instead of sending its bytes through the dataflow. One node registers it, writes into it in place, and publishes something small on a normal output — a slot index, a frame counter — to say that new data is there. Consumers map the segment once and read it where it lies.

This is for large payloads that are rewritten over and over: camera frames above all. For anything else, `send_output` is simpler and already zero-copy for messages of 4 KiB or more.

Three headers are involved, and only the first is required:

| Header | Contents | Requires |
|--------|----------|----------|
| `dora-node-api.h` | the pool functions themselves | nothing beyond the normal node build |
| `dora/memory_pool.hpp` | `dora::PoolWriteGuard`, `dora::PoolReadGuard`, `dora::try_read_pool` | `dora-node-api.h`; no CUDA |
| `dora/cuda_pool.hpp` | `dora::cuda::map_pool` / `unmap_pool` / `is_integrated_gpu` | the CUDA runtime, linked by your node |

Both extra headers are installed next to the generated `dora-node-api.h`. `dora/cuda_pool.hpp` is not included by anything — a node that does not include it needs no CUDA toolchain, and neither does the Rust build that generates the bridge.

A runnable end-to-end example, including the CUDA and Python-interop variants, is in [`examples/c++-memory-pool`](https://github.com/dora-rs/dora/tree/main/examples/c%2B%2B-memory-pool).

### Registering a pool

```cpp
#include "dora-node-api.h"
#include <dora/memory_pool.hpp>

auto dora_node = init_dora_node();

DoraMemoryPoolSpec spec{};
spec.id = rust::String("frames");     // your name for it; consumers ask by this
spec.size = 16 * 1024;                // bytes of payload
spec.dtype = rust::String("uint8");   // must not be empty
spec.shape = rust::Vec<std::size_t>();
spec.shape.push_back(16);             // must not be empty either
spec.shape.push_back(1024);
spec.transport = rust::String("auto");
spec.receiver_is_cuda = false;

// Throws on failure. The segment is unlinked again if the daemon refuses the
// registration, so a failed call leaves nothing behind in /dev/shm.
rust::Box<DoraMemoryPool> pool = register_memory_pool(dora_node.send_output, spec);
```

`dtype` and `shape` must both be non-empty — the daemon rejects a registration without them — and `shape` multiplied out by `dtype`'s element size must fit in `size`.

They are, however, **advisory metadata and never a bound**. An unrecognized `dtype` is assumed to be one byte per element, so a product computed from `shape` can fall short of the real extent; on a pool created by another binding it can also overshoot the mapping by megabytes. Size every write, every read and every view from `pool_payload_len()` / `view_payload_len()` instead. That is the number this API guarantees.

### Writing a frame

The pool has one writer at a time, bracketed by a seqlock so readers can tell a complete frame from one being overwritten. `dora::PoolWriteGuard` opens that bracket in its constructor and closes it in its destructor:

```cpp
{
    dora::PoolWriteGuard write(pool);
    if (write.size() < frame_bytes) {
        return;                          // early exit: the guard still closes
    }
    std::memcpy(write.data(), frame, frame_bytes);
    write.commit();                      // publishes the frame
}   // an uncommitted guard closes the cycle as *incomplete* instead
```

Use the guard rather than the raw `pool_begin_write` / `pool_end_write` pair. Between those two calls the pool's generation is odd, which is how every reader knows the payload is torn — but a `return`, a `break` or a thrown exception in between leaves it odd *permanently*. The pool is then unreadable to every consumer until some later write closes a cycle, and nothing on the Rust side recovers it. The guard's destructor runs on all three of those paths.

Publishing is opt-in: only `commit()` marks the frame complete. There is deliberately no rollback — the payload is written in place, so by the time a write fails the previous frame is already gone. An abandoned cycle costs one frame, not the pool.

If you already hold the bytes in a buffer of your own, `write_memory_pool` copies them and brackets its own cycle:

```cpp
DoraResult r = write_memory_pool(pool, data);   // data.size() == pool_payload_len(pool)
```

The length must match exactly: a short write would leave the previous frame's bytes in the tail and publish them as complete. Do not call it while a `PoolWriteGuard` is alive on the same pool — the second cycle is refused. Fill in place through the guard, or copy through this; not both.

### Reading a pool

A consumer maps the pool by id. The daemon supplies the segment name — it is not derivable from the id — so this fails if the pool is unknown or already freed.

```cpp
rust::Box<DoraMemoryPoolView> view =
    read_memory_pool(dora_node.send_output, "frames");   // throws on failure
```

For a consumer that wants its own copy, `dora::try_read_pool` sizes the destination from the pool and reads it under the seqlock:

```cpp
std::vector<std::uint8_t> frame;
switch (dora::try_read_pool(view, frame)) {
    case dora::PoolReadOutcome::Copied:      consume(frame); break;
    case dora::PoolReadOutcome::Torn:        break;   // retry on the next event
    case dora::PoolReadOutcome::Unavailable: return;  // never succeeds; stop
}
```

Three outcomes rather than a bool because one of the two failures is permanent and the other is not, and telling them apart is the difference between a retry and an infinite loop. `Torn` means a writer was mid-frame — likely to succeed next event. `Unavailable` means there is no payload in this mapping and there never will be: the pool has been freed, or it is an `ipc` pool whose bytes live in device memory. Distinguish those two with `view_is_alive` and `view_transport`.

For a consumer that works on the payload where it lies — a CUDA kernel over the device alias, a `cv::Mat` header over the host pointer — `dora::PoolReadGuard` samples the generation and lets you re-check it afterwards:

```cpp
dora::PoolReadGuard read(view);            // throws if there is no payload here
auto result = analyse(read.data(), read.size());
if (!read.valid()) {
    return;   // the writer overwrote the frame mid-analysis: discard `result`
}
```

`valid()` is not advisory. Everything computed from `data()` before it returned true was computed from bytes a writer may have been overwriting, so it must be thrown away, not merely flagged. Nothing here can prevent a torn read — only detect one.

### Transports

| transport | what is in the segment | how the receiver reaches the payload |
|-----------|------------------------|--------------------------------------|
| `shmem` | the full data region | read the mapping directly (CPU receiver) |
| `unified` | the full data region | `cudaHostRegister(..., cudaHostRegisterMapped)` + `cudaHostGetDevicePointer` |
| `ipc` | header only | `cudaIpcOpenMemHandle` on `view_ipc_handle()` |

`transport: "auto"` (or an empty string) resolves to `unified` when `receiver_is_cuda` is set and to `shmem` otherwise. The `DORA_MEMORY_POOL_TRANSPORT` environment variable overrides `auto` and nothing else: an explicit `shmem` or `unified` in the spec is the node author's decision about its own buffer layout, and a deployment's environment must not silently rewrite it. An unrecognized value in the variable is an error rather than a silent fallback.

`unified` is the only mode that works on an **integrated GPU**, where `cudaIpcGetMemHandle` is unsupported. It also works on a discrete GPU — it is merely slower there than IPC.

The C++ binding does not *produce* `ipc` pools; requesting `transport: "ipc"` is rejected at registration, because exporting a handle would require CUDA inside the binding. It does *read* one: a pool registered by the Python binding on a discrete GPU arrives as a view with `view_ipc_present()` true and the 64-byte handle available from `view_ipc_handle()`. Every accessor that would otherwise hand out an address reports the absence instead — `view_payload()` returns false, `view_payload_len()` is 0, and `try_read_pool` returns `Unavailable`.

### CUDA

`dora/cuda_pool.hpp` does mapping and unmapping, and nothing else. Streams, kernels and any `cudaMemcpy` stay in your node.

```cpp
#include <dora/cuda_pool.hpp>

dora::cuda::MappedPool m;
std::uint64_t base = 0;
std::size_t bytes = 0;
if (view_mapping(view, base, bytes) &&
    dora::cuda::map_pool(reinterpret_cast<void *>(base), bytes, m)) {
    std::size_t offset = 0;
    if (view_payload_offset(view, offset)) {
        const void *device_payload = static_cast<char *>(m.device) + offset;
        // ... hand device_payload to a kernel ...
    }
}
```

Two rules matter here. The first is silent when broken; the second only costs you the performance the pool exists for.

**Register the whole segment, then offset to the payload.** `cudaHostRegister` needs a page-aligned address, and a pool's payload does not start on a page boundary — it starts at a 256-byte boundary after the header and the padded metadata. So the pair you register is the mapping base and the segment length (`view_mapping`, or `pool_shm_base` + `pool_segment_bytes` on the producer side), and you reach the payload by adding `view_payload_offset` / `pool_payload_offset` to `m.device`. Both offset functions are predicates that return false — leaving your variable untouched — for an `ipc` or freed pool. Check the return value: on false, whatever you initialized the offset to lands you in the segment's header instead of the payload, and a kernel writing through that pointer corrupts the segment without crashing.

**Register once per pool or view, never per frame.** `cudaHostRegister` walks and pins every page in the segment, which is slow relative to a frame tick on a segment sized for real payloads. Map when the pool or the view first appears, and reuse the `MappedPool` across every frame.

`dora::cuda::is_integrated_gpu()` reports whether device 0 shares memory with the host. False means "not known to be integrated", not "confirmed discrete" — it also covers a query that failed because there is no CUDA device at all.

### Freeing, and what a CUDA node must do first

Any node may free a pool, not only the one that registered it. The daemon unlinks the segment and notifies every node that touched it.

```cpp
DoraResult r = free_memory_pool(dora_node.send_output, std::move(pool));
```

On the consumer side, drain the notifications once per event-loop pass, from the same thread that calls `read_memory_pool`:

```cpp
for (const rust::String &id : take_freed_pools()) {
    if (std::string(id) == "frames") {
        dora::cuda::unmap_pool(m);   // CUDA nodes only, and *before* the next line
        view.reset();                // e.g. a std::optional<rust::Box<...>>
    }
}
```

`take_freed_pools()` marks the matching views dead before it returns, so no accessor can hand out a pointer into a released segment: `view_is_alive` goes false, `try_read_pool` returns `Unavailable`, and a `PoolReadGuard` constructor throws.

A node that mapped the segment for CUDA has one extra obligation, and Rust cannot discharge it — Rust has no idea which segments were handed to the driver. **`cudaHostUnregister` must happen while the mapping is still there**, which means before *both* of these:

- the segment is unlinked, which `take_freed_pools()` warns you about; and
- you drop the pool or view handle. Dropping the `rust::Box` unmaps the segment out from under a still-registered mapping exactly as an unlink does — so an ordinary "done with this view" path at shutdown needs the unmap first too, and nothing enforces that for you.

Dropping a handle unmaps but never unlinks. The daemon owns the segment's lifetime and removes it on `free_memory_pool`, at dataflow shutdown, or during its orphan sweep.

### Function reference

Producer side, on a `rust::Box<DoraMemoryPool>`:

| Function | Returns |
|----------|---------|
| `register_memory_pool(sender, spec)` | the pool; throws on failure |
| `pool_id` / `pool_shm_name` / `pool_transport` | `rust::String` |
| `pool_dtype` / `pool_shape` | advisory metadata — never a bound |
| `pool_payload(pool, out_ptr, out_len)` | `false` for an `ipc` pool, leaving both outputs untouched |
| `pool_payload_len(pool)` | bytes of payload in the mapping — **this** is the bound |
| `pool_payload_offset(pool, out_offset)` | payload offset within the mapping; `false` for `ipc` |
| `pool_shm_base` / `pool_segment_bytes` | the page-aligned base and length `cudaHostRegister` takes |
| `pool_ipc_present(pool)` | whether the segment carries a CUDA IPC handle |
| `pool_begin_write` / `pool_end_write` | the raw cycle — prefer `dora::PoolWriteGuard` |
| `pool_write_in_progress(pool)` | whether a cycle is open on this handle |
| `write_memory_pool(pool, data)` | copies an exactly-`pool_payload_len()`-sized buffer in |
| `free_memory_pool(sender, std::move(pool))` | asks the daemon to release the pool |

Consumer side, on a `rust::Box<DoraMemoryPoolView>`:

| Function | Returns |
|----------|---------|
| `read_memory_pool(sender, pool_id)` | the view; throws on failure |
| `view_is_alive(view)` | `false` once a free notification has been drained |
| `view_id` / `view_shm_name` / `view_transport` | `rust::String` |
| `view_dtype` / `view_shape` | advisory metadata — never a bound, in either direction |
| `view_payload(view, out_ptr, out_len)` | `false` for an `ipc` or freed pool |
| `view_payload_len(view)` | bytes of payload in the mapping — **this** is the bound |
| `view_payload_offset(view, out_offset)` | payload offset within the mapping; `false` for `ipc` or freed |
| `view_mapping(view, out_base, out_bytes)` | the page-aligned base and length `cudaHostRegister` takes |
| `view_ipc_present` / `view_ipc_handle` | whether the segment carries a CUDA IPC handle, and the 64-byte handle itself; both report absent once the pool is freed |
| `view_try_read(view, dst)` | raw copy; `dst` must be exactly `view_payload_len()` bytes |
| `view_begin_read` / `view_read_valid` | the raw seqlock bracket — prefer `dora::PoolReadGuard` |
| `take_freed_pools()` | ids freed by any node since the last call |
