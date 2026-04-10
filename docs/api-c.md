# C API Reference

This document covers the two C APIs provided by the Dora framework: the **Node API** for standalone C processes and the **Operator API** for shared-library operators loaded by the Dora runtime.

## Table of Contents

- [Node API (dora-node-api-c)](#node-api-dora-node-api-c)
  - [Initialization](#initialization)
  - [Event Loop](#event-loop)
  - [Event Inspection](#event-inspection)
  - [Output](#output)
  - [Logging](#logging)
  - [Enums](#enums)
- [Operator API (dora-operator-api-c)](#operator-api-dora-operator-api-c)
  - [Lifecycle Functions](#lifecycle-functions)
  - [Event Handling](#event-handling)
  - [Input Reading](#input-reading)
  - [Output Sending](#output-sending)
  - [Memory Management](#memory-management)
  - [Structs](#structs)
  - [Enums](#operator-enums)
- [Node Example](#node-example)
- [Operator Example](#operator-example)
- [Building and Linking](#building-and-linking)

---

## Node API (dora-node-api-c)

Header: [`apis/c/node/node_api.h`](../apis/c/node/node_api.h)
Crate: `dora-node-api-c` (builds as `staticlib`)

The Node API is used by standalone C executables that participate in an Dora dataflow as external processes. The daemon spawns the process and sets environment variables that the node reads during initialization.

### Initialization

#### `init_dora_context_from_env`

```c
void *init_dora_context_from_env();
```

Initializes an Dora node context from environment variables set by the daemon. Returns an opaque pointer to the context on success, or `NULL` on failure.

The returned pointer must be passed to all subsequent Node API calls that expect a context argument. When the node is finished, free it with `free_dora_context`.

#### `free_dora_context`

```c
void free_dora_context(void *dora_context);
```

Frees a context previously created by `init_dora_context_from_env`. Each context must be freed exactly once. After freeing, the pointer must not be used again.

### Event Loop

#### `dora_next_event`

```c
void *dora_next_event(void *dora_context);
```

Blocks until the next event is available for this node. Returns an opaque pointer to the event, or `NULL` when all event streams have closed (indicating the node should exit).

The returned pointer must not be dereferenced directly. Use the `read_dora_*` functions to extract the event type and payload. Free the event with `free_dora_event` when done.

#### `free_dora_event`

```c
void free_dora_event(void *dora_event);
```

Frees an event previously returned by `dora_next_event`. Each event must be freed exactly once. After freeing, the event pointer and all derived pointers (from `read_dora_input_id`, `read_dora_input_data`) become invalid.

### Event Inspection

#### `read_dora_event_type`

```c
enum DoraEventType read_dora_event_type(void *dora_event);
```

Returns the type of the given event. See [DoraEventType](#doraeventtype) for possible values.

#### `read_dora_input_id`

```c
void read_dora_input_id(void *dora_event, char **out_ptr, size_t *out_len);
```

Reads the input ID from an `DoraEventType_Input` event. Writes the string start pointer to `*out_ptr` and its byte length to `*out_len`. The string is valid UTF-8 but **not** null-terminated; use `out_len` to determine its bounds.

If the event is not an input event, sets `*out_ptr = NULL` and `*out_len = 0`.

The returned pointer borrows from the event. It becomes invalid after `free_dora_event` is called.

#### `read_dora_input_data`

```c
void read_dora_input_data(void *dora_event, char **out_ptr, size_t *out_len);
```

Reads the raw data bytes from an `DoraEventType_Input` event. Writes the data start pointer to `*out_ptr` and its byte length to `*out_len`.

Sets `*out_ptr = NULL` and `*out_len = 0` if the event is not an input event or the input carries no data.

Currently only `UInt8` Arrow arrays are supported. Other Arrow data types will cause a runtime panic. Future versions will use the Arrow C Data Interface for full type support.

The returned pointer borrows from the event. It becomes invalid after `free_dora_event` is called.

#### `read_dora_input_timestamp`

```c
unsigned long long read_dora_input_timestamp(void *dora_event);
```

Returns the hybrid logical clock timestamp from an input event's metadata as a `uint64` value. Returns `0` if the event is not an input event.

### Output

#### `dora_send_output`

```c
int dora_send_output(
    void *dora_context,
    const char *id_ptr,
    size_t id_len,
    const char *data_ptr,
    size_t data_len
);
```

Sends output data to all downstream subscribers. The output ID (`id_ptr`/`id_len`) must be a valid UTF-8 string matching one of the node's declared outputs in the dataflow YAML. The data (`data_ptr`/`data_len`) is sent as raw bytes (UInt8 Arrow array).

Returns `0` on success, `-1` on error. Errors are logged via `tracing`.

Returns `-1` immediately if any pointer argument is `NULL`.

### Logging

#### `dora_log`

```c
int dora_log(
    void *dora_context,
    const char *level_ptr,
    size_t level_len,
    const char *msg_ptr,
    size_t msg_len
);
```

Sends a structured log message through the Dora logging pipeline. Both `level` and `msg` must be valid UTF-8 strings.

Valid log levels: `"error"`, `"warn"`, `"info"`, `"debug"`, `"trace"`.

Returns `0` on success, `-1` on error. Returns `-1` immediately if any pointer argument is `NULL`.

### Enums

#### `DoraEventType`

```c
enum DoraEventType {
    DoraEventType_Stop,        // Graceful shutdown requested
    DoraEventType_Input,       // New input data available
    DoraEventType_InputClosed, // An input stream was closed
    DoraEventType_Error,       // An error occurred
    DoraEventType_Unknown,     // Unrecognized event type
};
```

---

## Operator API (dora-operator-api-c)

Headers: [`apis/c/operator/operator_api.h`](../apis/c/operator/operator_api.h), [`apis/c/operator/operator_types.h`](../apis/c/operator/operator_types.h)
Crate: `dora-operator-api-c`

The Operator API is used by shared libraries (`.so`/`.dylib`/`.dll`) loaded into the Dora runtime process. Unlike nodes, operators do not have their own `main` function. Instead, they export three functions that the runtime calls at the appropriate lifecycle points.

The `operator_types.h` header is auto-generated by `safer-ffi` and defines all C-compatible struct and enum types.

### Lifecycle Functions

#### `dora_init_operator`

```c
DoraInitResult_t dora_init_operator(void);
```

Called once when the runtime loads the operator. Allocate and initialize any operator state, then return it via the `operator_context` field. The runtime passes this pointer back on every subsequent call.

Return an `DoraInitResult_t` with `.result.error = NULL` on success.

#### `dora_drop_operator`

```c
DoraResult_t dora_drop_operator(void *operator_context);
```

Called once when the operator is being unloaded. Free all resources associated with `operator_context`.

Return an `DoraResult_t` with `.error = NULL` on success.

### Event Handling

#### `dora_on_event`

```c
OnEventResult_t dora_on_event(
    RawEvent_t *event,
    const SendOutput_t *send_output,
    void *operator_context
);
```

Called by the runtime each time an event arrives for this operator. Inspect the `event` fields to determine the event type:

| Field | Meaning |
|-------|---------|
| `event->input != NULL` | New input available |
| `event->stop == true` | Graceful shutdown requested |
| `event->error.ptr != NULL` | An error occurred (UTF-8 string in `error.ptr`/`error.len`) |
| `event->input_closed.ptr != NULL` | An input stream closed (input ID in `input_closed.ptr`/`input_closed.len`) |

Use `send_output` to emit data to downstream nodes (see `dora_send_operator_output`). Return an `OnEventResult_t` with the appropriate `DoraStatus_t` to control the operator lifecycle.

### Input Reading

#### `dora_read_input_id`

```c
char *dora_read_input_id(const Input_t *input);
```

Returns a newly allocated null-terminated string containing the input ID. The caller must free it with `dora_free_input_id`.

#### `dora_read_data`

```c
Vec_uint8_t dora_read_data(Input_t *input);
```

Reads the input data as a byte array. Consumes the underlying Arrow array from the input (the data can only be read once per event). Returns a `Vec_uint8_t` with `.ptr = NULL` if the input has no data or the data has already been consumed.

The caller must free the returned data with `dora_free_data`.

### Output Sending

#### `dora_send_operator_output`

```c
DoraResult_t dora_send_operator_output(
    const SendOutput_t *send_output,
    const char *id,
    const uint8_t *data_ptr,
    size_t data_len
);
```

Sends output data to downstream subscribers. The `id` must be a null-terminated string matching one of the operator's declared outputs. The data (`data_ptr`/`data_len`) is converted to a UInt8 Arrow array internally.

Returns an `DoraResult_t` with `.error = NULL` on success.

### Memory Management

The Operator API allocates memory that the caller must free using the corresponding functions:

| Allocation source | Free function |
|-------------------|---------------|
| `dora_read_input_id` | `dora_free_input_id` |
| `dora_read_data` | `dora_free_data` |

```c
void dora_free_input_id(char *input_id);
void dora_free_data(Vec_uint8_t data);
```

Failing to call these functions will leak memory. Do not use `free()` on these allocations -- they are allocated by the Rust runtime and must be freed through the API.

### Structs

#### `Vec_uint8_t`

```c
typedef struct Vec_uint8 {
    uint8_t *ptr;
    size_t len;
    size_t cap;
} Vec_uint8_t;
```

A Rust-allocated byte vector. Access `len` bytes starting at `ptr`. Do not modify `cap`. Free with `dora_free_data`.

#### `DoraResult_t`

```c
typedef struct DoraResult {
    Vec_uint8_t *error;  // NULL on success, points to error string on failure
} DoraResult_t;
```

Generic result type. A `NULL` error pointer indicates success. When non-NULL, the error pointer contains a UTF-8 error message.

#### `DoraInitResult_t`

```c
typedef struct DoraInitResult {
    DoraResult_t result;
    void *operator_context;  // opaque pointer to operator state
} DoraInitResult_t;
```

Returned by `dora_init_operator`. On success, `result.error` is `NULL` and `operator_context` holds the operator state pointer.

#### `OnEventResult_t`

```c
typedef struct OnEventResult {
    DoraResult_t result;
    DoraStatus_t status;
} OnEventResult_t;
```

Returned by `dora_on_event`. Contains both an error/success result and a status code controlling the operator lifecycle.

#### `RawEvent_t`

```c
typedef struct RawEvent {
    Input_t *input;           // non-NULL when this is an input event
    Vec_uint8_t input_closed; // non-empty when an input stream closed
    bool stop;                // true when shutdown is requested
    Vec_uint8_t error;        // non-empty on error
} RawEvent_t;
```

Represents an event delivered to the operator. Multiple fields may be set simultaneously; check them in order of priority.

#### `Input_t`

```c
typedef struct Input Input_t;  // opaque
```

Opaque type representing an input event's data. Use `dora_read_input_id` and `dora_read_data` to extract its contents.

#### `Output_t`

```c
typedef struct Output Output_t;  // opaque
```

Opaque type used internally by `dora_send_operator_output`. Not created directly by user code.

#### `SendOutput_t`

```c
typedef struct SendOutput {
    ArcDynFn1_DoraResult_Output_t send_output;
} SendOutput_t;
```

Callback handle passed to `dora_on_event`. Pass it to `dora_send_operator_output` to emit data. Do not store it beyond the scope of the current `dora_on_event` call.

#### `Metadata_t`

```c
typedef struct Metadata {
    Vec_uint8_t open_telemetry_context;
} Metadata_t;
```

Event metadata containing an OpenTelemetry trace context string.

### Operator Enums

#### `DoraStatus_t`

```c
enum DoraStatus {
    DORA_STATUS_CONTINUE = 0,  // Keep running
    DORA_STATUS_STOP     = 1,  // Stop this operator
    DORA_STATUS_STOP_ALL = 2,  // Stop the entire dataflow
};
typedef uint8_t DoraStatus_t;
```

Returned in `OnEventResult_t` to control operator lifecycle after processing an event.

---

## Node Example

A complete C node that receives timer ticks and sends output messages:

```c
#include <stdio.h>
#include <string.h>
#include "node_api.h"

int main() {
    void *ctx = init_dora_context_from_env();
    if (ctx == NULL) {
        fprintf(stderr, "failed to init dora context\n");
        return 1;
    }

    for (int i = 0; i < 100; i++) {
        void *event = dora_next_event(ctx);
        if (event == NULL)
            break;  // all streams closed

        enum DoraEventType ty = read_dora_event_type(event);

        if (ty == DoraEventType_Input) {
            char *id;
            size_t id_len;
            read_dora_input_id(event, &id, &id_len);

            // Send a response
            char out_id[] = "message";
            char out_data[64];
            int out_len = snprintf(out_data, sizeof(out_data),
                                   "iteration %d", i);

            dora_send_output(ctx, out_id, strlen(out_id),
                              out_data, out_len);
        } else if (ty == DoraEventType_Stop) {
            free_dora_event(event);
            break;
        }

        free_dora_event(event);
    }

    free_dora_context(ctx);
    return 0;
}
```

Dataflow YAML for the node:

```yaml
nodes:
  - id: c_node
    path: build/c_node
    inputs:
      timer: dora/timer/millis/100
    outputs:
      - message
```

## Operator Example

A complete C operator that reads input, maintains state, and sends output:

```c
#include "operator_api.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

DoraInitResult_t dora_init_operator(void) {
    // Allocate operator state (a simple counter)
    int *counter = (int *)calloc(1, sizeof(int));

    DoraInitResult_t result = {.operator_context = counter};
    return result;
}

DoraResult_t dora_drop_operator(void *operator_context) {
    free(operator_context);
    DoraResult_t result = {.error = NULL};
    return result;
}

OnEventResult_t dora_on_event(
    RawEvent_t *event,
    const SendOutput_t *send_output,
    void *operator_context)
{
    OnEventResult_t result = {.status = DORA_STATUS_CONTINUE};
    int *counter = (int *)operator_context;

    if (event->input != NULL) {
        char *id = dora_read_input_id(event->input);
        Vec_uint8_t data = dora_read_data(event->input);

        if (data.ptr != NULL) {
            *counter += 1;
            printf("received input '%s', counter: %d\n", id, *counter);

            // Send counter value as string
            char buf[64];
            int len = snprintf(buf, sizeof(buf), "count=%d", *counter);
            result.result = dora_send_operator_output(
                send_output, "counter", (uint8_t *)buf, len);

            dora_free_data(data);
        }

        dora_free_input_id(id);
    }

    if (event->stop) {
        result.status = DORA_STATUS_STOP;
    }

    return result;
}
```

Dataflow YAML for the operator:

```yaml
nodes:
  - id: runtime-node
    operators:
      - id: c_operator
        shared-library: build/operator
        inputs:
          data: source_node/output
        outputs:
          - counter
```

---

## Building and Linking

### Node (static library)

C nodes link against `dora-node-api-c`, which builds as a static library.

**Step 1: Build the static library**

```bash
cargo build -p dora-node-api-c --release
```

This produces `target/release/libdora_node_api_c.a` (or `.lib` on Windows).

**Step 2: Compile and link**

```bash
clang node.c -ldora_node_api_c -L ../../target/release -o build/c_node <FLAGS>
```

Platform-specific linker flags:

| Platform | Flags |
|----------|-------|
| Linux | `-lm -lrt -ldl -pthread` |
| macOS | `-framework CoreServices -framework Security -lSystem -lresolv -lpthread -lc -lm` |
| Windows | `-ladvapi32 -luserenv -lkernel32 -lws2_32 -lbcrypt -lncrypt -lschannel -lntdll -liphlpapi -lcfgmgr32 -lcredui -lcrypt32 -lcryptnet -lfwpuclnt -lgdi32 -lmsimg32 -lmswsock -lole32 -lopengl32 -lsecur32 -lshell32 -lsynchronization -luser32 -lwinspool -Wl,-nodefaultlib:libcmt -D_DLL -lmsvcrt` |

On Windows, add the `.exe` extension to the output file.

### Operator (shared library)

C operators are compiled into shared libraries that the Dora runtime loads at startup.

**Step 1: Compile to object file**

```bash
clang -c operator.c -o build/operator.o -fdeclspec -fPIC
```

Omit `-fPIC` on Windows.

**Step 2: Link as shared library**

```bash
# Linux
clang -shared build/operator.o -o build/liboperator.so

# macOS
clang -shared build/operator.o -o build/liboperator.dylib

# Windows
clang -shared build/operator.o -o build/operator.dll
```

**Step 3: Reference in dataflow YAML**

```yaml
operators:
  - id: c_operator
    shared-library: build/operator   # without lib prefix or extension
    inputs:
      data: source/output
    outputs:
      - result
```

The `shared-library` path omits the platform-specific prefix (`lib`) and extension (`.so`/`.dylib`/`.dll`). The runtime resolves the correct file for the current platform.

### Include Paths

The Node API header is at `apis/c/node/node_api.h`. The Operator API headers are at `apis/c/operator/operator_api.h` and `apis/c/operator/operator_types.h`. Adjust your include paths accordingly:

```bash
# Node
clang -I path/to/dora/apis/c/node node.c ...

# Operator
clang -I path/to/dora/apis/c/operator operator.c ...
```

### C++ Compatibility

Both headers include `extern "C"` guards (in the operator headers) or use C-compatible declarations (in the node header), so they can be included directly from C++ source files.
