# Changelog

## Unreleased

### Breaking

- **`dora-operator-api-cxx` operator interface gains `on_input_closed` and `on_stop`**: previously the C++ operator API silently dropped `Event::InputClosed { id }` and `Event::Stop` via a catch-all `_ => Continue` arm — operators had no way to react to upstream input closure or graceful shutdown. The cxx::bridge now declares two additional callbacks that the C++ side must implement:
  ```cpp
  DoraOnInputResult on_input_closed(Operator& op, rust::Str id, OutputSender& output_sender);
  DoraOnInputResult on_stop(Operator& op, OutputSender& output_sender);
  ```
  Both receive the same per-event `OutputSender` as `on_input`, so operators can emit a final/status output in response to the event (e.g. flush buffered state on stop, send a "drain complete" marker on input close). Without these symbols, downstream C++ operators that build against `dora-operator-api-cxx` will fail to link. To restore pre-change behavior in an existing operator, add stubs that ignore `output_sender` and return `{ rust::String(), false }`. See [#1849](https://github.com/dora-rs/dora/pull/1849) (rescue of [#1414](https://github.com/dora-rs/dora/pull/1414)).
- **Python ABI floor bumped to `abi3-py311`** (from `abi3-py37`). The published `dora-rs` Python wheel now requires Python 3.11 or newer. This unblocks the new `send_output_raw` zero-copy send API (which depends on the stable buffer-protocol slots that became part of the C API in 3.11) and aligns with the 1.0 RC consolidation. Python 3.7, 3.8, 3.9 are all EOL; Python 3.10 EOLs October 2026. Users on Python 3.7–3.10 can stay on dora 0.5.0 during the transition. See [#1291](https://github.com/dora-rs/dora/pull/1291) / [#1833](https://github.com/dora-rs/dora/pull/1833).

### Added

- **`dora clean` subcommand**: removes fully-completed dataflows from the coordinator's state without restarting the coordinator. Candidates are enumerated from BOTH the in-memory `dataflow_results` map and the persisted store, so a restarted coordinator can still reap historical Succeeded/Failed rows that exist only on disk (the recovery loop intentionally does not reload completed dataflows into memory, so without this they would otherwise sit in redb forever). The coordinator deletes each cleaned dataflow from the persisted store first, then drops it from in-memory state; the redb deletion cascades to every `dora param` row owned by the cleaned dataflow so the on-disk state file doesn't grow unboundedly. Multi-daemon dataflows still finishing (some daemons reported, others haven't) are intentionally skipped so their final status is computed correctly when the last daemon completes. `finished_builds` is intentionally NOT touched (would break concurrent `dora build` calls with "unknown build id" errors). When the persisted-store delete fails for one or more candidates, the CLI prints a `warning:` line per failure to stderr and exits non-zero — those dataflows keep their in-memory entries so a later `dora clean` can retry, and the partial-outage state stays visible to scripted callers instead of being silently logged. If the persisted-store enumeration itself fails (e.g. an unreadable redb file), the coordinator hard-fails the request and leaves all in-memory state untouched, so the CLI never reports a misleading "nothing to clean" while historical rows are still on disk. Useful for dev workflows that accumulate dataflow history and for long-lived coordinators trimming redb state. Cleaned dataflows are no longer queryable via `dora logs <uuid>` or `dora param`. Rescue of [#1366](https://github.com/dora-rs/dora/pull/1366) (reimplemented against the current coordinator since the original PR targeted a control plane that was rewritten).
- **`node.send_output_raw(output_id, length, metadata=...)` — Python zero-copy send**: returns a `SampleHandler` that exposes dora's pre-allocated send buffer via Python's buffer protocol. The caller writes data directly into the buffer (`memoryview`, `numpy.asarray`, `struct.pack_into`) and either calls `.send()` or uses the context-manager form which sends on `__exit__`. Removes the one copy that `send_output(bytes_or_arrow)` performs to move Python-owned data into dora's send buffer — meaningful for camera frames, point clouds, and other MB-scale payloads at high rates. The handler enforces a safety contract: send-only-once, no view acquisition after `send()`, and `send()` refuses while buffer views are still open. See `examples/python-zero-copy-send/` for a runnable example. Rescue of [#1291](https://github.com/dora-rs/dora/pull/1291) / [#1833](https://github.com/dora-rs/dora/pull/1833).
- **Managed Python environments with `--uv`**: `dora build --uv` now creates a dedicated `uv` virtual environment per Python node at `<working-dir>/.dora/python-envs/<node-id>/` (for `.py` custom nodes with a `build:` block and runtime nodes with Python operators). Build commands run inside that venv with `VIRTUAL_ENV` set and the env's `bin/` (or `Scripts/` on Windows) prepended to `PATH`. `dora start` / `dora run` automatically reuse the same interpreter and runtime env at spawn time — build-time deps == runtime deps deterministically, no cross-dataflow or cross-node contamination, and subprocesses spawned from inside the node (`subprocess.run(["pip", ...])`, console scripts, `python -m pip`) all resolve from the managed env. Script-only Python nodes (no `build:` block) keep using the caller's ambient `uv` environment. Fail-closed under `--uv` when a managed env was expected but not recorded (e.g. stale session or non-`--uv` build), pointing users at `dora build --uv`. Rescue of [#1515](https://github.com/dora-rs/dora/pull/1515). See [#1820](https://github.com/dora-rs/dora/pull/1820).
- **`dora doctor` `uv` availability check**: surfaces `PASS uv: <version>` when `uv` is on PATH, `WARN` with install hint otherwise. WARN-not-FAIL because pure Rust/C++ users do not need `uv`. Part of [#1820](https://github.com/dora-rs/dora/pull/1820).
- **MAVLink 2 bridge** (`dora-mavlink2-bridge` extension + `dora-mavlink2-bridge-node` binary): Apache Arrow ↔ MAVLink 2 conversion for the common dialect (HEARTBEAT, SYS_STATUS, SYSTEM_TIME, ATTITUDE, ATTITUDE_QUATERNION, LOCAL_POSITION_NED, GLOBAL_POSITION_INT, GPS_RAW_INT, RC_CHANNELS, SERVO_OUTPUT_RAW, COMMAND_LONG, COMMAND_ACK, MISSION_CURRENT). TCP/UDP/serial transports, daemon-spawnable bridge node, and a self-contained `examples/mavlink2-bridge` dataflow with an in-process UDP simulator (no SITL/MAVProxy required). The example ships three consumer variants — Rust (`dataflow-rust.yml`), Python (`dataflow-python.yml`, `--uv`), and C++ (`dataflow-cxx.yml` via `cargo run --example mavlink2-bridge-cxx`) — all reading the same `bridge/heartbeat` Arrow output. See [#1786](https://github.com/dora-rs/dora/issues/1786).
- **`examples/mavlink2-bridge-sitl-mission`**: closed-loop ArduCopter SITL demo. A Python dora node arms, takes off, hovers, lands, and disarms a simulated multirotor by driving the bridge's `command_long_cmd` input and watching `command_ack` + `global_position_int`. Local-only on Ubuntu / macOS; not part of CI (SITL needs a one-time ArduPilot install per developer machine). See `examples/mavlink2-bridge-sitl-mission/README.md`.

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
