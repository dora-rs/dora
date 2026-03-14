# C++ API Parity Roadmap

Tracking document for bringing the dora C++ node/operator API up to parity with the Rust and Python APIs.
Relates to [issue #993](https://github.com/dora-rs/dora/issues/993).

## Status Legend
- [ ] Not started
- [~] In progress
- [x] Complete

## Dev Environment

Build and test inside Docker:
```bash
docker build -t dora-dev -f docker/dev/Dockerfile docker/dev/
docker run --rm -v $(pwd):/app -w /app dora-dev bash

# Inside container:
cargo check --package dora-node-api-cxx
cargo run --example cxx-dataflow          # basic C++ integration test
cargo run --example cxx-arrow-dataflow    # Arrow + metadata test (needs libarrow-dev)
cargo fmt --all -- --check
cargo clippy --package dora-node-api-cxx
```

---

## Phase 1: Node Accessors (PR 1)
**Goal:** Expose basic node introspection — the simplest possible PR to build maintainer trust.
**Files:** `apis/c++/node/src/lib.rs`

- [x] `node_id(sender) -> String` — return the node's ID from the dataflow config
- [x] `dataflow_id(sender) -> String` — return the running dataflow instance UUID

**Tests:**
- [x] Unit: `cargo check --package dora-node-api-cxx` — compiles with new CXX bridge functions
- [x] Integration (C++ to C++): `cargo run --example cxx-dataflow` — `node-rust-api/main.cc` asserts `node_id() == "cxx-node-rust-api"` and `dataflow_id()` is non-empty
- [x] Integration (Rust to C++): The dataflow chains Rust timer -> C++ node -> C++ node -> C++ operator, validating cross-language data flow with the new accessors printed in logs
- [ ] Integration (Python to C++): Add a Python sender node to `cxx-dataflow` that sends data to the C++ node (Phase 2 scope)

---

## Phase 2: Event Receive Variants (PR 2)
**Goal:** Unlock real-time C++ applications that cannot block indefinitely on `next()`.
**Files:** `apis/c++/node/src/lib.rs`

- [ ] `next_event_timeout(events, timeout_ms: u64) -> Box<DoraEvent>` — blocking recv with timeout
- [ ] `try_next_event(events) -> Box<DoraEvent>` — non-blocking, returns immediately
- [ ] `events_is_empty(events) -> bool` — check if events are pending
- [ ] `drain_events(events) -> Vec<DoraEvent>` — collect all buffered events

**Tests:**
- [ ] Unit: compile check for new CXX bridge functions
- [ ] Integration (C++ standalone): New example node that uses `try_next_event` in a polling loop with sleep, validates timeout returns empty event
- [ ] Integration (Python to C++): Python node sends burst of messages, C++ node uses `drain()` to collect them all, asserts count matches
- [ ] Integration (Rust to C++): Rust node sends on timer, C++ node uses `next_event_timeout` with short timeout, validates it returns after timeout with no event when timer hasn't fired

---

## Phase 3: Close Outputs + Missing Event Types (PR 3)
**Goal:** Full event lifecycle support.
**Files:** `apis/c++/node/src/lib.rs`

- [ ] `close_outputs(sender, output_ids: Vec<String>)` — signal downstream that outputs are done
- [ ] Add `NodeFailed` and `Reload` to `DoraEventType` enum
- [ ] Update `event_type()` match arms for new variants

**Tests:**
- [ ] Unit: compile check, event_type match exhaustiveness
- [ ] Integration (C++ to C++): C++ node calls `close_outputs("counter")`, downstream C++ node receives `InputClosed` event for that input
- [ ] Integration (Python to C++): Python node closes output, C++ node receives `InputClosed`

---

## Phase 4: Node Config & Descriptor (PR 4)
**Goal:** Let C++ nodes introspect their configuration and the full dataflow graph.
**Files:** `apis/c++/node/src/lib.rs`

- [ ] `node_config_json(sender) -> String` — JSON-serialized node run config
- [ ] `dataflow_descriptor_json(sender) -> String` — JSON-serialized full descriptor

**Tests:**
- [ ] Unit: compile check
- [ ] Integration (C++ standalone): Node parses returned JSON, validates it contains expected input/output IDs from the dataflow YAML
- [ ] Integration (Rust to C++): Compare `node_config` output from a Rust helper node against C++ node's output in same dataflow

---

## Phase 5: Operator API Parity (PR 5)
**Goal:** C++ operators should handle all event types, not just `Input`.
**Files:** `apis/c++/operator/src/lib.rs`

- [ ] Forward `Stop`, `InputClosed`, `Error` events to the C++ operator
- [ ] Add Arrow data support to operator inputs
- [ ] Update operator example

**Tests:**
- [ ] Unit: compile check for updated operator bridge
- [ ] Integration (C++ operator): Updated example operator logs all event types, validates Stop is received at shutdown
- [ ] Integration (Python to C++ operator): Python node sends Arrow data, C++ operator receives it via Arrow interface

---

## Phase 6: Dynamic Node Init (PR 6)
**Goal:** Enable C++ nodes spawned dynamically outside YAML dataflow definitions.
**Files:** `apis/c++/node/src/lib.rs`

- [ ] `init_dora_node_by_id(node_id: &str) -> Result<DoraNode>`

**Tests:**
- [ ] Unit: compile check
- [ ] Integration: Dynamic C++ node registers with coordinator, exchanges messages with a YAML-defined Python node

---

## Phase 7 (Stretch): Zero-Copy & Async
- [ ] `allocate_data_sample(len) -> DataSample` — shared memory allocation
- [ ] Async recv (C++ coroutines or callback-based)

**Tests:**
- [ ] Benchmark: Compare throughput of zero-copy vs copy path for large payloads (>4KB)
- [ ] Integration: Async recv with callback validates events are delivered

---

## API Gap Matrix

| Capability               | Rust | Python | C++ | Target PR |
|---------------------------|------|--------|-----|-----------|
| Init from env             | Yes  | Yes    | Yes | —         |
| Init from node_id         | Yes  | Yes    | No  | Phase 6   |
| Send output (bytes)       | Yes  | Yes    | Yes | —         |
| Send output (Arrow)       | Yes  | Yes    | Yes | —         |
| Send output w/ metadata   | Yes  | Yes    | Yes | —         |
| Metadata read/write       | Yes  | Yes    | Yes | —         |
| Merge external events     | Yes  | Yes    | Yes | —         |
| `node_id()`               | Yes  | No     | **Yes** | Phase 1 |
| `dataflow_id()`           | Yes  | Yes    | **Yes** | Phase 1 |
| `recv_timeout()`          | Yes  | Yes    | No  | Phase 2   |
| `try_recv()`              | Yes  | Yes    | No  | Phase 2   |
| `drain()`                 | Yes  | Yes    | No  | Phase 2   |
| `is_empty()`              | Yes  | Yes    | No  | Phase 2   |
| `close_outputs()`         | Yes  | No     | No  | Phase 3   |
| `NodeFailed` event        | Yes  | No     | No  | Phase 3   |
| `Reload` event            | Yes  | No     | No  | Phase 3   |
| `node_config()`           | Yes  | Yes    | No  | Phase 4   |
| `dataflow_descriptor()`   | Yes  | Yes    | No  | Phase 4   |
| Operator: all events      | Yes  | Yes    | No  | Phase 5   |
| Zero-copy buffers         | Yes  | No     | No  | Phase 7   |
