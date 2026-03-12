# C++ API Reference

Adora provides C++ bindings for both standalone nodes and in-process operators via [CXX](https://cxx.rs/) (Rust-C++ interop). The CXX bridge generates type-safe C++ headers from Rust definitions -- no raw FFI or manual `extern "C"` declarations are needed.

Two crates provide the C++ surface:

| Crate | Library | Use case |
|-------|---------|----------|
| `adora-node-api-cxx` | `libadora_node_api_cxx.a` | Standalone node executable |
| `adora-operator-api-cxx` | `libadora_operator_api_cxx.a` | Shared-library operator loaded by the runtime |

Generated headers: `adora-node-api.h` and `adora-operator-api.h`.

---

## Node API (`adora-node-api-cxx`)

### Initialization

```cpp
#include "adora-node-api.h"

// Initialize a node from environment variables set by the Adora daemon.
// Returns an AdoraNode struct containing the event stream and output sender.
// Throws on failure.
AdoraNode init_adora_node();
```

### AdoraNode

Returned by `init_adora_node()`. Owns the event stream and the output sender for the lifetime of the node.

```cpp
struct AdoraNode {
    rust::Box<Events>        events;       // event stream (blocking receiver)
    rust::Box<OutputSender>  send_output;  // output sender
};
```

### Events

Opaque Rust type exposed to C++. Provides blocking iteration over the node's incoming events.

```cpp
// Member function -- call on the boxed object directly.
rust::Box<AdoraEvent> Events::next();

// Free function form -- equivalent to events->next().
rust::Box<AdoraEvent> next_event(rust::Box<Events>& events);
```

Both forms block until the next event arrives and return an owned `AdoraEvent`.

### AdoraEvent

Opaque Rust type. Inspect its kind with `event_type()`, then downcast with `event_as_input()` or `event_as_arrow_input()`.

```cpp
// Determine the event kind.
AdoraEventType event_type(const rust::Box<AdoraEvent>& event);

// Downcast to a raw-byte input. Throws if the event is not Input.
AdoraInput event_as_input(rust::Box<AdoraEvent> event);

// Downcast to an Arrow FFI input (writes Arrow C Data Interface structs).
// out_array and out_schema must point to valid ArrowArray / ArrowSchema structs.
// Returns AdoraResult with empty error on success.
AdoraResult event_as_arrow_input(
    rust::Box<AdoraEvent> event,
    uint8_t* out_array,
    uint8_t* out_schema);

// Same as above, but also returns the input ID and metadata.
ArrowInputInfo event_as_arrow_input_with_info(
    rust::Box<AdoraEvent> event,
    uint8_t* out_array,
    uint8_t* out_schema);
```

### AdoraEventType

```cpp
enum class AdoraEventType : uint8_t {
    Stop,             // graceful shutdown requested
    Input,            // new data arrived on an input
    InputClosed,      // a single input was closed
    Error,            // an error occurred
    Unknown,          // unrecognized event variant
    AllInputsClosed,  // all inputs closed (stream ended)
};
```

### AdoraInput

Returned by `event_as_input()`. Contains raw bytes.

```cpp
struct AdoraInput {
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

### AdoraResult

Returned by output-sending functions. Check the `error` field -- empty means success.

```cpp
struct AdoraResult {
    rust::String error;  // empty string on success
};
```

### OutputSender

Opaque Rust type. All methods take `rust::Box<OutputSender>&` as the first argument (the sender from `AdoraNode::send_output`).

#### send_output

Send raw bytes on a named output.

```cpp
AdoraResult send_output(
    rust::Box<OutputSender>& sender,
    rust::String id,
    rust::Slice<const uint8_t> data);
```

#### send_output_with_metadata

Send raw bytes with attached metadata.

```cpp
AdoraResult send_output_with_metadata(
    rust::Box<OutputSender>& sender,
    rust::String id,
    rust::Slice<const uint8_t> data,
    rust::Box<Metadata> metadata);
```

#### send_arrow_output

Send an Arrow array via the C Data Interface. The pointers must reference valid `ArrowArray` and `ArrowSchema` structs. Ownership of the Arrow data transfers to Rust on success.

```cpp
AdoraResult send_arrow_output(
    rust::Box<OutputSender>& sender,
    rust::String id,
    uint8_t* array_ptr,
    uint8_t* schema_ptr);

// Overload with metadata (same C++ name via cxx_name attribute).
AdoraResult send_arrow_output(
    rust::Box<OutputSender>& sender,
    rust::String id,
    uint8_t* array_ptr,
    uint8_t* schema_ptr,
    rust::Box<Metadata> metadata);
```

#### log_message

Send a log message through the Adora logging system.

```cpp
AdoraResult log_message(
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
// Convert Adora events into a combined stream.
CombinedEvents adora_events_into_combined(rust::Box<Events> events);

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
    rust::Box<MergedAdoraEvent> event;

    bool is_adora() const;  // true if this is a standard Adora event
};

// Downcast a combined event back to an AdoraEvent. Throws if not an Adora event.
rust::Box<AdoraEvent> downcast_adora(CombinedEvent event);
```

ROS2 subscriptions add their own events to the merged stream. Use `subscription->matches(event)` and `subscription->downcast(event)` to handle ROS2-specific events (see the [ROS2 Bridge docs](ros2-bridge.md)).

---

## Operator API (`adora-operator-api-cxx`)

Operators are shared libraries loaded by the Adora runtime. The C++ side implements two functions that the CXX bridge calls into.

### Required C++ interface

You must provide a header `operator.h` and an implementation file. The header declares an `Operator` class and two free functions:

```cpp
// operator.h
#pragma once
#include <memory>
#include "adora-operator-api.h"

class Operator {
public:
    Operator();
    // Add any state your operator needs.
};

std::unique_ptr<Operator> new_operator();

AdoraOnInputResult on_input(
    Operator& op,
    rust::Str id,
    rust::Slice<const uint8_t> data,
    OutputSender& output_sender);
```

- `new_operator()` -- called once at startup; returns the operator instance.
- `on_input()` -- called for every input event; process data and optionally send outputs.

### OutputSender (operator)

Available inside `on_input()`. Sends data on a named output.

```cpp
AdoraSendOutputResult send_output(
    OutputSender& sender,
    rust::Str id,
    rust::Slice<const uint8_t> data);
```

### Result types

```cpp
struct AdoraOnInputResult {
    rust::String error;  // empty on success
    bool         stop;   // true to request graceful shutdown
};

struct AdoraSendOutputResult {
    rust::String error;  // empty on success
};
```

---

## Quick Start: Node Example

A minimal node that receives timer ticks and sends a counter.

```cpp
#include "adora-node-api.h"
#include <iostream>
#include <vector>

int main() {
    auto adora_node = init_adora_node();
    unsigned char counter = 0;

    for (;;) {
        auto event = next_event(adora_node.events);
        auto ty = event_type(event);

        if (ty == AdoraEventType::AllInputsClosed) {
            break;
        }
        if (ty == AdoraEventType::Stop) {
            break;
        }
        if (ty == AdoraEventType::Input) {
            auto input = event_as_input(std::move(event));
            counter += 1;

            std::cout << "Input: " << std::string(input.id)
                      << " counter=" << (int)counter << std::endl;

            std::vector<unsigned char> out{counter};
            rust::Slice<const uint8_t> slice{out.data(), out.size()};
            auto result = send_output(adora_node.send_output, "counter", slice);
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
      tick: adora/timer/millis/300
    outputs:
      - counter
```

---

## Quick Start: Arrow Node Example

A node that receives and sends Arrow arrays via the C Data Interface, with metadata.

```cpp
#include "adora-node-api.h"
#include <arrow/api.h>
#include <arrow/c/bridge.h>
#include <iostream>

int main() {
    auto adora_node = init_adora_node();

    for (int i = 0; i < 10; i++) {
        auto event = adora_node.events->next();
        auto ty = event_type(event);

        if (ty == AdoraEventType::AllInputsClosed || ty == AdoraEventType::Stop) {
            break;
        }
        if (ty == AdoraEventType::Input) {
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
                adora_node.send_output, "counter",
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

AdoraOnInputResult on_input(
    Operator& op,
    rust::Str id,
    rust::Slice<const uint8_t> data,
    OutputSender& output_sender)
{
    op.counter += 1;

    std::vector<unsigned char> out{op.counter};
    rust::Slice<const uint8_t> slice{out.data(), out.size()};
    auto send_result = send_output(output_sender, rust::Str("status"), slice);

    return AdoraOnInputResult{send_result.error, false};
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
link_directories(${adora_link_dirs})

# Standalone node (executable)
add_executable(my_node node/main.cc ${node_bridge})
add_dependencies(my_node Adora_cxx)
target_include_directories(my_node PRIVATE ${adora_cxx_include_dir})
target_link_libraries(my_node adora_node_api_cxx)

# Operator (shared library)
add_library(my_operator SHARED
    operator/operator.cc ${operator_bridge})
add_dependencies(my_operator Adora_cxx)
target_include_directories(my_operator PRIVATE
    ${adora_cxx_include_dir} ${adora_c_include_dir}
    ${CMAKE_CURRENT_SOURCE_DIR}/operator)
target_link_libraries(my_operator adora_operator_api_cxx)

install(TARGETS my_node DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/bin)
install(TARGETS my_operator DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/lib)
```

### What DoraTargets.cmake provides

| Variable | Description |
|----------|-------------|
| `adora_cxx_include_dir` | Path to generated CXX headers (`adora-node-api.h`, `adora-operator-api.h`) |
| `adora_c_include_dir` | Path to C API headers (for mixed C/C++ projects) |
| `adora_link_dirs` | Library search path for `libadora_node_api_cxx.a` / `libadora_operator_api_cxx.a` |
| `node_bridge` | Generated CXX bridge source file for nodes (`node_bridge.cc`) |
| `operator_bridge` | Generated CXX bridge source file for operators (`operator_bridge.cc`) |
| `Adora_cxx` | CMake target dependency that builds the CXX crates |

### Build steps

```bash
# Option A: Build against local Adora source
mkdir build && cd build
cmake .. -DDORA_ROOT_DIR=/path/to/adora
cmake --build .

# Option B: Build against Adora from GitHub (cloned automatically)
mkdir build && cd build
cmake ..
cmake --build .
```

### Requirements

- C++20 compiler
- Rust toolchain (for building the Adora static libraries via Cargo)
- CMake 3.21+
- For Arrow integration: Apache Arrow C++ library

---

## CXX Bridge Notes

- All Rust opaque types (`Events`, `OutputSender`, `AdoraEvent`, `Metadata`, `MergedEvents`, `MergedAdoraEvent`) are accessed through `rust::Box<T>`.
- `rust::String`, `rust::Vec<T>`, and `rust::Slice<const T>` are CXX bridge types that interoperate with their C++ standard library counterparts. See the [CXX type reference](https://cxx.rs/binding/box.html).
- Functions that return `Result<T>` in Rust throw C++ exceptions on the error path.
- Arrow FFI functions (`event_as_arrow_input`, `send_arrow_output`) are `unsafe` on the Rust side. The caller must pass valid pointers to `ArrowArray` / `ArrowSchema` structs cast to `uint8_t*`.
- The node library is a static archive (`staticlib`). Link it into your executable with `-ladora_node_api_cxx`.
- The operator library is also a static archive. Link it into your shared library with `-ladora_operator_api_cxx`.
