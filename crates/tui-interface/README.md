# TUI Interface Crate

This crate hosts the shared data contracts and service abstractions between the Dora framework and
the TUI. It is the implementation detail of [ADR-001](../../docs/tui-boundary/adr-001-tui-interface-boundary.md).

> **Status:** scaffolding. The modules currently expose placeholder types so the boundary can be
> built incrementally.

## Modules

- `data`: DTOs that describe dataflows, nodes, telemetry, and user preferences consumed by the TUI.
- `services`: traits the TUI depends on (coordinator access, legacy command execution, telemetry,
  preference persistence).
- `error`: shared error types used by the traits.

Future work will move the real implementations/structs into this crate and add contract tests.
