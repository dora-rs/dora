# Changelog

## Unreleased

### Breaking

- **`dora-operator-api-cxx` operator interface gains `on_input_closed` and `on_stop`**: previously the C++ operator API silently dropped `Event::InputClosed { id }` and `Event::Stop` via a catch-all `_ => Continue` arm — operators had no way to react to upstream input closure or graceful shutdown. The cxx::bridge now declares two additional callbacks that the C++ side must implement:
  ```cpp
  DoraOnInputResult on_input_closed(Operator& op, rust::Str id, OutputSender& output_sender);
  DoraOnInputResult on_stop(Operator& op, OutputSender& output_sender);
  ```
  Both receive the same per-event `OutputSender` as `on_input`, so operators can emit a final/status output in response to the event (e.g. flush buffered state on stop, send a "drain complete" marker on input close). Without these symbols, downstream C++ operators that build against `dora-operator-api-cxx` will fail to link. To restore pre-change behavior in an existing operator, add stubs that ignore `output_sender` and return `{ rust::String(), false }`. See [#1849](https://github.com/dora-rs/dora/pull/1849) (rescue of [#1414](https://github.com/dora-rs/dora/pull/1414)).
- **Coordinator persisted-store `SCHEMA_VERSION` bumped `2` → `3`**: `DataflowStatus::Failed` gained a `terminal` field (#1854) without a matching schema bump, so schema `2` has silently meant two different, mutually incompatible on-disk record shapes: `v1.0.0-rc.1` stores (no `terminal`) and `v1.0.0-rc.2` stores (`terminal` included). bincode is not self-describing, so `#[serde(default)]` cannot backfill a missing trailing field the way it would for JSON — an rc.1-shaped `Failed` record read under the current type either fails to decode or (since `status` is not the last field of `DataflowRecord`) silently mis-decodes every field after it, and `list_dataflows`/`get_dataflow` were swallowing that failure with only a `warn!` log, so terminally-failed dataflows could silently vanish from `dora list` after a coordinator restart. `RedbStore::open()` now refuses to open **any** v2-stamped database with a `schema version mismatch` error — including rc.2 stores, whose row shape happens to still decode correctly, because schema `2` is ambiguous and the two shapes cannot be told apart from the version number alone. **If you have an existing coordinator store from `v1.0.0-rc.1` or `v1.0.0-rc.2`** (`~/.dora/` by default; `redb` is the default `--store` backend), `dora coordinator` will refuse to start after upgrading past this change. Delete the store file (its path is named in the error message) to start fresh, or pass `--store memory` to bypass persistence. See [#2471](https://github.com/dora-rs/dora/issues/2471).
- **Coordinator persisted-store `SCHEMA_VERSION` bumped `3` → `4`**: `DataflowRecord` gained `ready_barrier_released` and `barrier_exited_before_subscribe`, which persist whether a dataflow's start barrier has been released and with what verdict. The coordinator needs this because the in-memory `RunningDataflow` does not survive orphan reclaim (every daemon running the dataflow disconnecting) or a coordinator restart, and a daemon that missed the original `AllNodesReady` broadcast cannot ask for another — daemon-side `reported_init_to_coordinator` is set once and never reset — so without a durable record of the release its nodes park in `init_from_env()` for the life of the dataflow (#2998). As with the `2` → `3` bump above, bincode is positional rather than self-describing, so `#[serde(default)]` cannot backfill the added fields: a v3-shaped record read under the current type shifts every field after `uv` and mis-decodes into a structurally valid but wrong record. `RedbStore::open()` therefore rejects a v3-stamped database with a `schema version mismatch` error. **If you have an existing coordinator store written before this change** (`~/.dora/` by default; `redb` is the default `--store` backend), `dora coordinator` will refuse to start after upgrading. Delete the store file (its path is named in the error message) to start fresh, or pass `--store memory` to bypass persistence. See [#2998](https://github.com/dora-rs/dora/issues/2998) / [#3013](https://github.com/dora-rs/dora/pull/3013).
- **Python ABI floor bumped to `abi3-py311`** (from `abi3-py37`). The published `dora-rs` Python wheel now requires Python 3.11 or newer. This unblocks the new `send_output_raw` zero-copy send API (which depends on the stable buffer-protocol slots that became part of the C API in 3.11) and aligns with the 1.0 RC consolidation. Python 3.7, 3.8, 3.9 are all EOL; Python 3.10 EOLs October 2026. Users on Python 3.7–3.10 can stay on dora 0.5.0 during the transition. See [#1291](https://github.com/dora-rs/dora/pull/1291) / [#1833](https://github.com/dora-rs/dora/pull/1833).

### Added

- **Memory-pool transport for the C++ node binding** (`dora-node-api-cxx`): `register_memory_pool` / `write_memory_pool` / `read_memory_pool` / `free_memory_pool`, plus direct pointer access under an explicit seqlock so a producer can fill the shared segment in place instead of copying a buffer into it. Two optional headers install next to the generated bridge header. `dora/memory_pool.hpp` is CUDA-free and wraps the write and read cycles in `PoolWriteGuard` / `PoolReadGuard` — the guard matters because an early return or a thrown exception between the raw `pool_begin_write` / `pool_end_write` pair leaves the pool's generation odd permanently, which makes it unreadable to every consumer — and adds `try_read_pool`, whose three-valued outcome separates a retryable torn read from a pool that can never be read again. `dora/cuda_pool.hpp` is header-only and does `cudaHostRegister(..., cudaHostRegisterMapped)` + `cudaHostGetDevicePointer` over the segment's page-aligned mapping base (never the payload pointer, which starts at a 256-byte boundary inside the segment). No Rust crate links CUDA: the CUDA edge stays on the C++ side, as it does in the Python binding. `dtype` and `shape` are advisory metadata throughout — `pool_payload_len` / `view_payload_len` is the only bound for a copy. See `examples/c++-memory-pool` for a runnable CPU dataflow plus CUDA and Python-interop variants.
- **`unified` memory-pool transport**: a full shared-memory data region with no CUDA IPC handle, which a CUDA receiver reaches through the host-pinned device alias rather than `cudaIpcOpenMemHandle`. This is the only mode that works on an integrated GPU, where `cudaIpcGetMemHandle` is unsupported — the Python binding's `register_memory_pool` fails outright there for a CUDA receiver, since a receiver's header-only segment is useless without the handle. `transport: "auto"` resolves to `unified` when the spec sets `receiver_is_cuda` and to `shmem` otherwise; `DORA_MEMORY_POOL_TRANSPORT` overrides `auto` and nothing else, so an explicit choice in the spec is never rewritten by the environment. The C++ binding does not *produce* `ipc` pools (that would need CUDA inside the binding) but it reads one, exposing the handle via `view_ipc_handle`. No protocol change was needed: `unified` uses the same segment *layout* the Python binding already writes — the same `DORADMA` header, the seqlock word at the same offset, the padded metadata region, and a data region at the header's `data_offset`. The two are **not** byte-identical (a Python CPU pool writes no `transport` key, writes `pinned_type: "cpu"`, uses `json.dumps` spacing and so a different `json_len`, and lands at `write_gen = 2` where the C++ side creates at `0`); they interoperate because the consumer reads each of those fields out of the header instead of assuming it. The C++ side reads and writes `DORADMA` through one definition of the format, in `dora-memory-pool`. The Python binding is unchanged, and still carries its own hand-written copy of the header constants and field offsets in `apis/python/node/src/lib.rs`, so this change *adds* a second definition rather than consolidating the existing one. A test in `dora-memory-pool` reads Python's three constants out of that file and fails if they drift; the offsets Python writes by hand are literals with no names to read, and are not covered. Folding the Python binding onto `dora-memory-pool` is follow-up work. Setting `DORA_MEMORY_POOL_TRANSPORT=unified` on a C++ producer registers `pinned_type: "cuda"`, which a Python *consumer* reads as "take the CUDA path" — so the override is not free when the consumer is a CPU Python node. See [#2686](https://github.com/dora-rs/dora/issues/2686).
- **`dora clean` subcommand**: removes fully-completed dataflows from the coordinator's state without restarting the coordinator. Candidates are enumerated from BOTH the in-memory `dataflow_results` map and the persisted store, so a restarted coordinator can still reap historical Succeeded/Failed rows that exist only on disk (the recovery loop intentionally does not reload completed dataflows into memory, so without this they would otherwise sit in redb forever). The coordinator deletes each cleaned dataflow from the persisted store first, then drops it from in-memory state; the redb deletion cascades to every `dora param` row owned by the cleaned dataflow so the on-disk state file doesn't grow unboundedly. Multi-daemon dataflows still finishing (some daemons reported, others haven't) are intentionally skipped so their final status is computed correctly when the last daemon completes. `finished_builds` is intentionally NOT touched (would break concurrent `dora build` calls with "unknown build id" errors). When the persisted-store delete fails for one or more candidates, the CLI prints a `warning:` line per failure to stderr and exits non-zero — those dataflows keep their in-memory entries so a later `dora clean` can retry, and the partial-outage state stays visible to scripted callers instead of being silently logged. If the persisted-store enumeration itself fails (e.g. an unreadable redb file), the coordinator hard-fails the request and leaves all in-memory state untouched, so the CLI never reports a misleading "nothing to clean" while historical rows are still on disk. Useful for dev workflows that accumulate dataflow history and for long-lived coordinators trimming redb state. Cleaned dataflows are no longer queryable via `dora logs <uuid>` or `dora param`. Rescue of [#1366](https://github.com/dora-rs/dora/pull/1366) (reimplemented against the current coordinator since the original PR targeted a control plane that was rewritten).
- **`node.send_output_raw(output_id, length, metadata=...)` — Python zero-copy send**: returns a `SampleHandler` that exposes dora's pre-allocated send buffer via Python's buffer protocol. The caller writes data directly into the buffer (`memoryview`, `numpy.asarray`, `struct.pack_into`) and either calls `.send()` or uses the context-manager form which sends on `__exit__`. Removes the one copy that `send_output(bytes_or_arrow)` performs to move Python-owned data into dora's send buffer — meaningful for camera frames, point clouds, and other MB-scale payloads at high rates. The handler enforces a safety contract: send-only-once, no view acquisition after `send()`, and `send()` refuses while buffer views are still open. See `examples/python-zero-copy-send/` for a runnable example. Rescue of [#1291](https://github.com/dora-rs/dora/pull/1291) / [#1833](https://github.com/dora-rs/dora/pull/1833).
- **Managed Python environments with `--uv`**: `dora build --uv` now creates a dedicated `uv` virtual environment per Python node at `<working-dir>/.dora/python-envs/<node-id>/` (for `.py` custom nodes with a `build:` block and runtime nodes with Python operators). Build commands run inside that venv with `VIRTUAL_ENV` set and the env's `bin/` (or `Scripts/` on Windows) prepended to `PATH`. `dora start` / `dora run` automatically reuse the same interpreter and runtime env at spawn time — build-time deps == runtime deps deterministically, no cross-dataflow or cross-node contamination, and subprocesses spawned from inside the node (`subprocess.run(["pip", ...])`, console scripts, `python -m pip`) all resolve from the managed env. Script-only Python nodes (no `build:` block) keep using the caller's ambient `uv` environment. Fail-closed under `--uv` when a managed env was expected but not recorded (e.g. stale session or non-`--uv` build), pointing users at `dora build --uv`. Rescue of [#1515](https://github.com/dora-rs/dora/pull/1515). See [#1820](https://github.com/dora-rs/dora/pull/1820).
- **`dora doctor` `uv` availability check**: surfaces `PASS uv: <version>` when `uv` is on PATH, `WARN` with install hint otherwise. WARN-not-FAIL because pure Rust/C++ users do not need `uv`. Part of [#1820](https://github.com/dora-rs/dora/pull/1820).
- **MAVLink 2 bridge** (`dora-mavlink2-bridge` extension + `dora-mavlink2-bridge-node` binary): Apache Arrow ↔ MAVLink 2 conversion for the common dialect (HEARTBEAT, SYS_STATUS, SYSTEM_TIME, ATTITUDE, ATTITUDE_QUATERNION, LOCAL_POSITION_NED, GLOBAL_POSITION_INT, GPS_RAW_INT, RC_CHANNELS, SERVO_OUTPUT_RAW, COMMAND_LONG, COMMAND_ACK, MISSION_CURRENT). TCP/UDP/serial transports, daemon-spawnable bridge node, and a self-contained `examples/mavlink2-bridge` dataflow with an in-process UDP simulator (no SITL/MAVProxy required). The example ships three consumer variants — Rust (`dataflow-rust.yml`), Python (`dataflow-python.yml`, `--uv`), and C++ (`dataflow-cxx.yml` via `cargo run --example mavlink2-bridge-cxx`) — all reading the same `bridge/heartbeat` Arrow output. See [#1786](https://github.com/dora-rs/dora/issues/1786).
- **`examples/mavlink2-bridge-sitl-mission`**: closed-loop ArduCopter SITL demo. A Python dora node arms, takes off, hovers, lands, and disarms a simulated multirotor by driving the bridge's `command_long_cmd` input and watching `command_ack` + `global_position_int`. Local-only on Ubuntu / macOS; not part of CI (SITL needs a one-time ArduPilot install per developer machine). See `examples/mavlink2-bridge-sitl-mission/README.md`.

### Fixed

- **Nodes no longer outlive a SIGKILLed `dora run`** (Unix): an in-process daemon (`dora run`, `dora daemon --run-dataflow`, and embedders driving `Daemon::run_dataflow`) now hands every node it spawns the run process's pid (`DORA_RUN_PARENT_PID`), and the node API's orphan guard — armed at the top of `init`, before anything that can block — SIGKILLs the node's **own process group** once that pid is gone. The whole group on purpose: under `--uv` the tracked process is a wrapper (`uv run python ...`) and ending only the caller would leave the interpreter orphaned one level down. Previously such nodes ran forever at `ppid 1` in a group of their own, unreachable by inherited signal delivery or a group-kill of the CLI, because nodes are deliberately spawned as process-group leaders and a node that is not polling its event stream never observes the daemon-socket EOF. Scoped to in-process daemons only: the `dora up` + `dora start` path injects nothing, so nodes there still survive daemon restarts (#2029). The variable is control-plane-denylisted — a descriptor cannot forge it and a stale shell export cannot leak it. Known gaps, tracked on the issue: a process killed before it reaches `init` (a Python node still in `import torch`), `path: shell` commands that never call `init`, and Windows (left to the daemon's Job Object wrap). See [#2856](https://github.com/dora-rs/dora/issues/2856) / [#3018](https://github.com/dora-rs/dora/pull/3018).

## 0.1.0 (2026-03-13)

First official release of Dora (AI-Dora) -- a 100% Rust framework for building real-time robotics and AI applications.

### Highlights

- **10-17x faster than ROS2 Python** with zero-copy shared memory IPC and Apache Arrow native data format
- **Multi-language support** -- write nodes in Rust, Python, C, or C++ with native APIs
- **Declarative YAML dataflows** -- define pipelines as directed graphs with typed inputs/outputs
- **Single CLI for full lifecycle** -- build, run, monitor, record/replay, and distributed deployment

### Core Framework

- Coordinator/daemon architecture for distributed multi-machine deployments
- WebSocket control plane (single port for CLI and topic data)
- Shared memory transport for messages >= 4KB (zero-copy)
- Zenoh pub-sub for cross-machine communication
- Apache Arrow columnar format throughout (zero serialization overhead)

### Fault Tolerance

- Per-node restart policies (never/on-failure/always) with exponential backoff
- Passive health monitoring for hung node detection
- Per-input circuit breakers with configurable timeouts and auto re-subscription
- Coordinator state persistence via redb

### Communication Patterns

- Topic (default pub/sub)
- Service (request/reply with correlation IDs)
- Action (goal/feedback/result with cancellation)
- Streaming (session/segment/chunk for voice and real-time apps)

### Logging and Observability

- Unified logging with `dora/logs` virtual input and OTel propagation
- Structured output with filtering, rotation, and log routing
- `dora top` TUI for per-node CPU, memory, queue depth, network I/O
- `topic echo/hz/info` for live data inspection
- `trace list/view` for coordinator span inspection
- Record/replay to `.drec` files with node substitution

### CLI

- `dora run` for local dev, `dora up/start` for distributed prod
- `dora record/replay` for dataflow debugging
- `dora cluster up/status/down` for SSH-based fleet management
- `dora node info/restart/stop`, `dora topic pub`
- `dora doctor` for environment diagnostics
- `dora status --json` for programmatic access
- Configurable queue policies (`drop_oldest` / `backpressure`) per input

### Module System

- Reusable sub-graphs as standalone YAML files
- Typed inputs/outputs with parameters and nested composition
- Optional type annotations with build-time static validation

### ROS2 Bridge

- Bidirectional topic, service, and action bridging
- Declarative YAML configuration
- Per-topic QoS override

### Developer Experience

- Hot reload for Python operators
- In-process operators for lightweight functions
- Comprehensive examples (15+ dataflows)
- mdBook user guide (English and Chinese)
- GitHub Actions CI with cross-platform testing

### Security

- Bearer token authentication (opt-in)
- Per-IP WebSocket rate limiting
- Path traversal guards, input validation, error sanitization
- Resource limits and bounded data structures
