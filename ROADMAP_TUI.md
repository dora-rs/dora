# TUI / New CLI Implementation Roadmap

## Milestone Overview
- Cleanly separate legacy CLI workflows from the emerging TUI so users have one syntax per experience.
- Replace mock data feeds in the TUI with live coordinator and telemetry data.
- Flesh out advanced views and persistence to graduate the hybrid experience from scaffolding to production-ready.

## Issue Tracker

### 1. Refine CLI Split Between Legacy & TUI
- **Branch**: `refactor/cli-split`
- **Key Files**: `binaries/cli/src/lib.rs`, `binaries/cli/src/cli/mod.rs`, `binaries/cli/src/main.rs`, `binaries/cli/src/tui/command_executor.rs`
- **Scope**: trim the hybrid parser so only true TUI entry points (`tui`, `dashboard`, future TUI-only verbs) remain; restore all traditional operations to the legacy parser; and provide an internal helper the TUI can call to reuse legacy commands for start/stop/logs, etc.
- **Plan**:
  - Remove or hide preview command variants from the new CLI that duplicate legacy verbs, and update help text accordingly.
  - Simplify `run_new_cli` dispatch paths to launch the TUI or dashboard only; route all other invocations through the legacy CLI.
  - Add a dedicated bridge API for the TUI command palette to trigger legacy commands without shelling out (e.g., `execute_legacy_command(&["start", ...])`).
  - Audit docs and tests so usage examples reflect the new split (`dora <cmd>` for CLI, `dora tui` for interactive mode).
- **Done When**: `dora tui` is the lone public entry point to the interactive experience, while `dora <cmd>` remains authoritative for all non-interactive workflows and the TUI can still perform operations internally via the bridge.

### 2. Hook TUI Command Palette To Real CLI Execution
- **Branch**: `feat/tui-command-execution`
- **Key Files**: `binaries/cli/src/tui/app.rs`, `binaries/cli/src/tui/command_executor.rs`, `binaries/cli/src/lib.rs`
- **Scope**: replace the mock `execute_cli_command` implementation with calls into the legacy-command bridge created in Issue 1; surface errors/success back into the TUI status area and persist command results for history view.
- **Done When**: issuing `:start`, `:stop`, `:logs`, etc. inside the TUI drives the actual operations via the shared bridge without spawning external processes.

### 3. Replace Mock Dataflow State With Coordinator Queries
- **Branch**: `feat/tui-live-dataflows`
- **Key Files**: `binaries/cli/src/tui/app.rs`, `binaries/cli/src/tui/views/dataflow_explorer.rs`, `binaries/cli/src/command/list.rs`
- **Scope**: reuse the coordinator RPC logic from the legacy `list` command so `AppState.dataflows` reflects real deployments. Update the explorer and dashboard to render the richer metadata and handle loading/error cases.
- **Done When**: the TUI’s dataflow list matches `dora list` output in real time.

### 4. Wire Real System Metrics Into Dashboard & Monitor
- **Branch**: `feat/tui-live-metrics`
- **Key Files**: `binaries/cli/src/tui/app.rs`, `binaries/cli/src/tui/views/system_monitor.rs`, telemetry modules under `libraries/extensions/telemetry`
- **Scope**: fetch CPU, memory, and network statistics from the telemetry layer or coordinator endpoints; update `SystemMetrics` and the dashboard charts; degrade gracefully when telemetry is unavailable.
- **Done When**: dashboard and system monitor show live numbers rather than placeholders.

### 5. Implement Node Inspector & Advanced Views
- **Branch**: `feat/tui-node-inspector`
- **Key Files**: `binaries/cli/src/tui/views/node_inspector.rs`, `dataflow_manager.rs`, `analysis_tools.rs`
- **Scope**: populate node-level metrics, inputs/outputs, and logs; remove placeholder comments from analysis/collaboration views; ensure navigation reacts to live state updates.
- **Done When**: selecting a node surfaces its real metrics/log stream, and advanced panes render actionable data.

### 6. Persist Command History & User Preferences
- **Branch**: `feat/tui-history-preferences`
- **Key Files**: `binaries/cli/src/tui/app.rs`, `binaries/cli/src/cli/config/preference_manager.rs`, `binaries/cli/src/cli/context.rs`
- **Scope**: implement on-disk history storage, load/save UI preferences via the configuration system, and resolve outstanding TODOs around pattern/context matching.
- **Done When**: TUI sessions remember history and UI settings across restarts.

### 7. Activate Advanced CLI Modules (Debug / Analyze / Monitor / System)
- **Branch**: `feat/new-cli-advanced-commands`
- **Key Files**: `binaries/cli/src/lib.rs`, `binaries/cli/src/debug`, `binaries/cli/src/analysis`, `binaries/cli/src/automation`
- **Scope**: connect the advanced commands parsed by the new CLI to their backend modules; refine output formatting; document the new workflows and fill in missing APIs where feasible.
- **Done When**: commands like `dora tui debug` and `dora tui analyze` produce meaningful diagnostics, and documentation/examples in `docs/` reflect the behavior.

---
- Each roadmap item follows the standard workflow: create issue → branch → commits → PR → review/merge.
- Reference TODOs with issue numbers as they are addressed (e.g., `TODO(#123)`), and keep docs/tests in sync.
- Run `cargo fmt`, targeted `cargo test`, and `cargo clippy` before opening each PR.
