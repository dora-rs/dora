# TUI ↔ Core Touchpoints Inventory

Date: 2025-02-14  
Issue: [#101](https://github.com/heyong4725/dora/issues/101)

> **Update (2025-03-28):** The Dora TUI now lives in the standalone repository
> [heyong4725/dora-tui](https://github.com/heyong4725/dora-tui). This document is retained to track
> historical touchpoints and ensure the remaining CLI code consumes the shared crates via git
> dependencies rather than local paths.

## Scope
This note captures every place where the TUI stack inside `binaries/cli/src/tui` reaches into
the rest of the Dora CLI / framework. Each dependency is classified so we can decide whether it
belongs in a future `tui-interface` crate (data contracts), a service abstraction, or should stay
local to the TUI.

## Summary

| TUI module | External dependency | Category | Notes / Boundary candidate |
| --- | --- | --- | --- |
| `binaries/cli/src/tui/app.rs` | `dora_core::topics::DORA_COORDINATOR_PORT_CONTROL_DEFAULT` | Data contract | Constant used when opening control-channel sockets. Likely belongs to shared interface crate. |
| | `dora_message::coordinator_to_cli::{DataflowListEntry, DataflowStatus, NodeRuntimeInfo, NodeRuntimeState}` | Data contract | Structures used to populate TUI dataflow state. Good candidates for shared crate re-export. |
| | `dora_message::descriptor::{CoreNodeKind, ResolvedNode}` | Data contract | Node metadata displayed in explorer/inspector; should flow through boundary DTOs. |
| | `crate::common::{connect_to_coordinator, query_running_dataflows}` | Behaviour / service call | Direct coordinator RPC helpers. Needs abstraction (e.g., trait in interface crate) before split. |
| | `crate::config::preferences::UserPreferences` | Behaviour / persistence | Used to load/save UI prefs. Consider exposing preference access via boundary API. |
| | `crate::LOCALHOST` | Data contract | Constant for control socket defaults. Could be duplicated or exposed through interface crate. |
| | `crate::cli::context::ExecutionContext::detect_basic` | Behaviour / environment | Used to capture shell working dir/env for legacy command execution. Should migrate into service factory when protocol boundary is in place. |
| `binaries/cli/src/tui/bridge.rs` | `crate::{common, config::preferences, execute_legacy_command}` | Service wiring | Aggregates coordinator, legacy CLI, telemetry, and preference helpers into a `ServiceBundle`. The bundle is still CLI-specific and will be replaced by protocol client registration. |
| `binaries/cli/src/tui/command_executor.rs` | `crate::cli::{Cli, Command, commands::*, context::ExecutionContext}` | Data contract + behaviour | Parses legacy CLI args and inspects command enums. Suggest exporting slim command facade via boundary crate to avoid deep CLI coupling. |
| | `crate::execute_legacy_command` via bridge | Behaviour bridge | Executes legacy CLI operations. Needs dedicated service trait (e.g., `LegacyCommandGateway`) so TUI can call into core without depending on full CLI implementation. |
| `binaries/cli/src/tui/cli_integration.rs` | `crate::cli::{Cli, Command}` | Data contract | Reuses CLI parser types when constructing `CliContext`. Same action item as above. |
| `binaries/cli/src/tui/views/node_inspector.rs` | `dora_message::descriptor::CoreNodeKind` | Data contract | Used to label node types; should be provided via boundary DTO. |
| `binaries/cli/src/tui/tests.rs` (tests only) | `crate::config::preferences::UserPreferences` | Behaviour / persistence | Test helper for config reload path. When boundary is enforced, tests should go through interface abstraction instead of direct config access. |
| `binaries/cli/src/tui/command_executor.rs` | `tokio::task::spawn_blocking` + legacy bundle | Concurrency | Executes legacy CLI commands asynchronously; protocol gateway will need equivalent async support. |

No other TUI modules import from outside the `tui` namespace; they rely solely on TUI-local types.

## Operation Inventory (P0 Audit)

| UI Command / Feature | Current Implementation Path | Data / Action Required | Protocol Considerations |
| --- | --- | --- | --- |
| `ps` (list dataflows) | `StateSynchronizer::refresh_dataflow_state` → `connect_to_coordinator` RPC | Retrieve `DataflowList` plus node runtime info | Expose `GET /dataflows` (support pagination and optional node expansion). |
| `start`, `stop`, `up`, `destroy` | `execute_legacy_and_refresh` dispatches to legacy CLI (`dora start …`) | Lifecycle actions on coordinator/daemon | Provide command verbs via protocol (`POST /dataflows/{id}:start`, etc.) and surface async status. |
| `build`, `new`, `check`, `graph` | Legacy CLI execution | Project scaffolding and validation | Decide whether protocol should forward to existing CLI or expose services (initial scope: allow passthrough). |
| `logs` | Legacy CLI command then TUI view switch | Tail / fetch logs for dataflow or system | Protocol needs streaming/log chunk endpoints with follow support. |
| `inspect` | Refreshes local cache then opens view | Node + dataflow metadata | Protocol should support `/dataflows/{id}` and `/dataflows/{id}/nodes/{node}` to hydrate inspector without direct coordinator RPC. |
| `debug`, `monitor`, `analyze`, `system`, `help` | Legacy CLI execution | Diagnostics, monitoring, documentation | Determine MVP scope: either expose protocol verbs or leave to CLI bridge until parity is needed. |
| TUI auto-refresh telemetry | `StateSynchronizer::refresh_system_metrics` → `TelemetryService::latest_metrics` (sysinfo collector) | System CPU/memory/network snapshots | Replace local sysinfo with telemetry service over protocol (`/telemetry/system` streaming). |
| Preferences management | `CliPreferencesStore` reads/writes `UserPreferences` on disk | Persist theme, refresh interval, hints | Provide protocol-backed preference service or remote storage fallback; ensure update events propagate. |
| Command mode execution | All legacy commands via `legacy_service.execute` | Executes arbitrary `dora` commands, updates status | Protocol client must handle generic command execution or restrict to supported verbs with clear feedback. |

### Gaps & TODOs Identified

- **Lifecycle APIs:** Need explicit protocol endpoints for start/stop/up/destroy with idempotent behaviour and progress reporting.
- **Log streaming:** Current implementation shells out to CLI; protocol must expose chunked or websocket log feeds.
- **Telemetry:** Replace local `sysinfo` collector with coordinator-driven metrics so TUI works remotely.
- **Preferences:** Abstract config persistence; consider storing preferences via coordinator or separate service to avoid disk coupling.
- **Command Palette:** Decide on supported commands vs legacy passthrough. For unsupported verbs, provide user feedback through protocol error codes.
- **Authentication/Context:** ExecutionContext currently infers working directory/env. Protocol should convey user/workspace context explicitly rather than relying on local environment.

## Detailed Findings

### `binaries/cli/src/tui/app.rs`
- Imports coordinator channel constants from `dora_core` and multiple dataflow/node structs from
  `dora_message` ([app.rs#L21-L31](../../binaries/cli/src/tui/app.rs#L21)).
- Calls `crate::common::connect_to_coordinator` and `query_running_dataflows` to refresh state and
  uses the global `LOCALHOST` constant ([app.rs#L37-L51](../../binaries/cli/src/tui/app.rs#L37)).
- Reads and writes `UserPreferences` to hydrate/persist UI settings ([app.rs` user_config` flows]).
- **Implication:** we need an interface crate that re-exports message DTOs and wraps coordinator RPC
  helpers behind a trait (`CoordinatorClient`). `UserPreferences` access should also be routed
  through an abstraction so future repo split can depend on a thin config API.

### `binaries/cli/src/tui/command_executor.rs`
- Depends heavily on CLI internals: parses `Cli`, matches on `Command`, uses `commands::*`, and
  builds an `ExecutionContext` ([command_executor.rs#L8-L29](../../binaries/cli/src/tui/command_executor.rs#L8)).
- Relies on `execute_legacy_command` to hand commands back to the legacy CLI implementation
  ([command_executor.rs#L103-L156](../../binaries/cli/src/tui/command_executor.rs#L103)).
- Uses the service bundle (`crate::tui::bridge::default_service_bundle`) to obtain coordinator,
  telemetry, and legacy command handles. The bundle is hard-wired to CLI helpers and will need to
  be replaced by protocol client registration.
- **Implication:** introduce a facade (e.g., `LegacyCliService`) that exposes high-level operations
  (`ps`, `start`, `stop`, etc.) so TUI does not reach into raw CLI enums. That facade can live in
  the shared crate and be implemented inside the core repo.

### `binaries/cli/src/tui/bridge.rs`
- Packs coordinator RPC (`connect_to_coordinator`), preference persistence (`UserPreferences`),
  telemetry sampling (`MetricsCollector`), and legacy command execution (`execute_legacy_command`)
  into a single `ServiceBundle`.
- The bridge currently assumes local access to coordinator sockets and file system. Once the
  protocol server exists, this module should register protocol clients instead of touching those
  helpers directly.

### `StateSynchronizer` & Telemetry
- Refreshes dataflow state through coordinator RPC and fetches system metrics via `TelemetryService`
  backed by the local `MetricsCollector` (sysinfo).
- **Implication:** telemetry must be served by the core/coordinator so remote TUIs can obtain
  consistent metrics; protocol needs either streaming or polling endpoints.

### `binaries/cli/src/tui/cli_integration.rs`
- Imports `Cli` and `Command` to capture launch context information
  ([cli_integration.rs#L9-L10](../../binaries/cli/src/tui/cli_integration.rs#L9)).
- **Implication:** same facade as above; `CliContext` should consume a simplified entry type rather
  than the entire CLI parser.

### `binaries/cli/src/tui/views/node_inspector.rs`
- Uses `dora_message::descriptor::CoreNodeKind` to display node categories
  ([node_inspector.rs#L17-L19](../../binaries/cli/src/tui/views/node_inspector.rs#L17)).
- **Implication:** Node details should arrive via the coordinator DTO returned by the boundary API,
  so the TUI code no longer grabs types directly from `dora_message`.

### `binaries/cli/src/tui/tests.rs`
- Test-only dependency on `UserPreferences` to verify reload behaviour
  ([tests.rs#L4-L20](../../binaries/cli/src/tui/tests.rs#L4)).
- **Implication:** when the boundary is enforced, swap these tests to use the same abstraction the
  runtime code will use.

## Next Steps
- Design `CoordinatorClient` + `LegacyCliService` traits inside the planned `tui-interface` crate.
- Move shared DTOs (dataflow summaries, node info, telemetry samples, user preferences snapshot) to
  that crate, re-export from Dora CLI for backwards compatibility.
- Adjust TUI modules to depend solely on the new crate, replacing the direct imports listed above.
- Stand up protocol gateway & client per roadmap issues (#113–#115) to replace direct coordinator
  calls, legacy command execution, telemetry sampling, and preference persistence.
- Define logging/streaming endpoints so `logs` and telemetry views can operate without shelling out
  to the legacy CLI.
