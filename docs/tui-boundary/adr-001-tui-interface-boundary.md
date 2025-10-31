# ADR-001: TUI ↔ Core Interface Boundary

- **Date:** 2025-02-14
- **Status:** Proposed
- **Related Issues:** [#101](https://github.com/heyong4725/dora/issues/101), [#103](https://github.com/heyong4725/dora/issues/103)
- **Decision Drivers:**
  - Reduce coupling between the Dora framework and the emerging TUI so each can evolve independently.
  - Prepare for a future repo split without blocking current development.
  - Provide a single, well-defined contract that both sides can implement and test against.

## Context

The TUI currently lives under `binaries/cli/src/tui` and reaches into many parts of the Dora CLI and
framework (legacy command parsing, coordinator RPC helpers, user preference persistence, etc.).
Issue #101 documented these touchpoints in [docs/tui-boundary/touchpoints.md](./touchpoints.md).

Without a boundary:
- Every framework refactor risks breaking the TUI.
- The TUI cannot be developed or released independently.
- Tests have to stand up large portions of the CLI to exercise TUI behaviour.

We want a gradual approach (Option 2) that keeps the code in this repo for now but enforces strict
interfaces. Later, we can spin the TUI out with minimal disruption.

## Decision

1. Introduce a new crate `crates/tui-interface` that owns shared data contracts and service traits.
2. Split responsibilities:
   - **Core framework/CLI provides** concrete implementations of the traits and exposes them to the
     TUI through dependency injection.
   - **TUI consumes** only the trait objects and DTOs from the interface crate; it no longer imports
     framework internals directly.
3. Enforce the boundary with lint guards, dependency checks, and targeted CI jobs.
4. Define versioning & compatibility for the interface crate (semver, deprecation policy, test
   strategy) so future repo separation is low-risk.

### Interface Crate Contents

| Category | Examples | Notes |
| --- | --- | --- |
| Data Contracts | `DataflowSummary`, `NodeSummary`, `SystemMetrics`, `UserPreferencesSnapshot` | Mirror the structures the TUI needs to render the UI. Avoid leaking raw `dora_message` types—convert them in the framework implementation. |
| Service Traits | `CoordinatorClient`, `LegacyCliService`, `PreferencesStore`, `TelemetryStream` | Define async methods for the operations TUI triggers (list dataflows, start/stop, fetch metrics, load/save prefs). Core CLI provides the live implementations. |
| Errors & Result Types | `InterfaceError` enum, `Result<T>` alias | Standardise error reporting across services. |
| Configuration | `InterfaceConfig` for tuning behaviour (timeouts, endpoint overrides). | Allows the TUI to be configured without referencing CLI globals. |

### Responsibility Split

| Responsibility | Core CLI / Framework | TUI |
| --- | --- | --- |
| Coordinator connectivity | Implements `CoordinatorClient` using existing RPC helpers (`connect_to_coordinator`, etc.). | Calls trait methods to fetch/update state; handles retries and UI messaging. |
| Legacy command execution | Exposes high-level methods (`ps`, `start`, `stop`, `logs`) via `LegacyCliService`. | Invokes service methods from command palette; no direct use of `Cli` parser or `Command` enums. |
| User preferences | Implements `PreferencesStore` to load/save data (reusing `UserPreferences`). | Uses trait to read initial config and persist changes. |
| Telemetry | Exposes realtime metrics via `TelemetryStream`. | Subscribes and renders metrics; falls back gracefully if unavailable. |
| DTO transformations | Converts framework types into interface DTOs. | Treats DTOs as read-only data. |

### Versioning & Compatibility

- The interface crate follows **semantic versioning**.
- Breaking changes require a major version bump and an ADR update.
- CLI/framework vendors this crate internally during the transition; when the TUI moves out-of-repo
  it will depend on the published crate.
- Introduce boundary tests:
  - Contract tests in the interface crate verify DTO serialization and trait defaults.
  - Integration tests in the TUI consume only trait objects (with mocks) to ensure independence.
  - End-to-end tests in the CLI instantiate the real implementations to catch regressions.

### Migration Plan (High-Level)

1. Extract DTOs & helper logic from TUI/framework into the new crate.
2. Implement service traits in the CLI, adapting existing helper functions.
3. Update TUI code to use only the interface crate.
4. Add dependency guardrails:
   - `#[deny(clippy::all)]` is insufficient; add custom `cargo check` script that fails if TUI code
     imports from forbidden modules.
   - Optionally add a compile-time guard macro (e.g., `use crate::forbidden::*;` to detect leaks).
5. Document public API in README + rustdoc; keep changelog to track interface evolution.

### Extended Roadmap: Protocol Boundary

To prepare for multi-client support (future standalone TUI, refreshed CLI, dashboards), we will
introduce a network-friendly protocol between user interfaces and the core runtime. This builds on
the `tui-interface` traits but formalises transport, versioning, and rollout.

| Phase | Issue/Track | Deliverables |
| --- | --- | --- |
| **P0. Discovery** | (#111) Protocol Touchpoint Audit | • Catalogue all UI→core operations (list/start/stop/logs/preferences/telemetry).<br>• Identify missing coordinator endpoints or data needed to satisfy UI use cases. |
| **P1. Protocol Design** | (#112) Protocol ADR | • Choose transport (initial proposal: JSON over HTTP/WebSocket, but gRPC is a candidate).<br>• Define message schemas, version negotiation, and auth placeholder.<br>• Decide on error codes and pagination/streaming semantics. |
| **P2. Server Adapter** | (#113) Coordinator Gateway | • Implement protocol server inside the coordinator (or a sidecar) that fulfils the trait contracts.<br>• Add feature flag to run alongside existing RPC layer.<br>• Add contract tests (golden requests/responses). |
| **P3. Client SDK** | (#114) Protocol Client Crate | • Generate or handwrite client bindings (Rust first).<br>• Provide async APIs mirroring `tui-interface` traits.<br>• Ship mocks/fakes for UI tests. |
| **P4. TUI Migration** | (#115) TUI Protocol Adoption | • Swap TUI service wiring to use the protocol client by default.<br>• Keep existing in-process trait implementations as fallback (feature flag).<br>• Add integration tests that start the protocol server and run UI smoke tests. |
| **P5. CLI Migration** | (#116) Legacy CLI Alignment | • Port `ps`, `start`, `stop`, `logs`, etc. to call through the protocol.<br>• Remove direct coordinator RPC access once parity is achieved.<br>• Update documentation and examples to reference the new API. |
| **P6. Harden & Publish** | (#117) Backward Compatibility & Release | • Establish protocol versioning policy (semantic version, compatibility matrix).<br>• Automate compatibility tests (client vN vs server vN±1).<br>• Document rollout playbook for future clients (web UI, external tooling). |

Interim milestones:
- Coordinate with operations/UX to validate that the protocol surface covers expected workflows.
- Ensure logging, tracing, and authentication hooks are defined even if initially no-op.
- Provide migration guides for third-party extensions once the protocol is public.

### Testing Strategy

- Mock implementations of `CoordinatorClient`, `LegacyCliService`, and `PreferencesStore` will live
  in the interface crate’s `tests/` directory for reuse by the TUI.
- The CLI integration test suite registers the real implementations and runs smoke tests to ensure
  the TUI contracts remain satisfied.
- Long term, these mocks can move to the future TUI repo so it can test independently.

## Consequences

**Positive**
- Clear separation of concerns, enabling faster TUI iteration and easier maintenance.
- Simplified path to a future repo split (Option 2 → Option 1).
- Reduced regression risk: boundary tests catch contract violations early.

**Negative / Risks**
- Initial refactoring cost: moving DTOs and abstracting services will touch many files.
- Requires discipline to keep the boundary crate minimal and avoid leaking new dependencies.
- Until the repo split happens, we add another crate to build/test.

**Mitigations**
- Tackle refactor incrementally per roadmap (Milestones 2 & 3).
- Automate guardrails to prevent boundary erosion.
- Keep ADR and roadmap updated as the interface evolves.

## References

- [Touchpoints Inventory](./touchpoints.md)
- Roadmap item “Option 2 – Gradual TUI Extraction”
