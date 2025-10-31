# Protocol Boundary Tracking Issues

Use the following checklist to open dedicated GitHub issues for each phase of the protocol roadmap
captured in ADR-001. Replace the placeholder `[issue-number]` when creating the real issues.

| ID | Title | Summary | Acceptance / Notes |
| --- | --- | --- | --- |
| #111 | Protocol Touchpoint Audit | Inventory all current TUI/CLI → core interactions (commands, telemetry, prefs). Update `docs/tui-boundary/touchpoints.md` with findings and explicit gaps. | ✅ Documented flow chart / table; ✅ list of missing API surface; ✅ added TODOs to roadmap. |
| #112 | Protocol Specification ADR | Draft protocol ADR (transport choice, message schema, error model, versioning). | ✅ ADR merged; ✅ schema examples; ✅ migration considerations noted. |
| #113 | Coordinator Protocol Gateway | Implement protocol server/gateway (likely within coordinator). Feature flag for roll-out. | ✅ Server builds/tests; ✅ smoke tests; ✅ docs describing endpoint config. |
| #114 | Rust Protocol Client SDK | Expose async client matching `tui-interface` traits incl. mocks for tests. | ✅ Crate published/internal; ✅ doc comment examples; ✅ unit tests with mock server. |
| #115 | TUI Protocol Adoption | Switch TUI to consume protocol client (bridge as fallback). | ✅ TUI builds using protocol; ✅ feature flag for legacy path; ✅ integration test hitting live gateway. |
| #116 | CLI Command Migration | Route legacy CLI commands through protocol client; retire direct coordinator RPC uses. | ✅ Commands (`ps`, `start`, etc.) call protocol; ✅ remove old helpers; ✅ updated docs/examples. |
| #117 | Protocol Compatibility & Release | Define versioning, integration tests (client vN vs server vN±1), publication process. | ✅ Compatibility matrix documented; ✅ automated test in CI; ✅ release checklist. |

## Suggested `gh issue create` Commands

Execute (and edit) the following commands from the repository root to populate GitHub issues:

```sh
gh issue create --title "Protocol Touchpoint Audit" \
  --body-file docs/tui-boundary/templates/111-protocol-touchpoint-audit.md \
  --label "proto-boundary,P0"

gh issue create --title "Design Protocol Specification" \
  --body-file docs/tui-boundary/templates/112-protocol-spec.md \
  --label "proto-boundary,P1"

# …repeat for remaining IDs…
```

> Tip: copy the issue body templates (see below) into `docs/tui-boundary/templates/` and tweak as
> needed before running the commands.

## Issue Body Templates (Draft)

Create the following files under `docs/tui-boundary/templates/` if you want reusable template text.

<details>
<summary><code>111-protocol-touchpoint-audit.md</code></summary>

```markdown
## Goal
- Catalogue every interaction the TUI and legacy CLI make with the core runtime/coordinator.
- Highlight any missing operations or data required by the UI.

## Deliverables
- Updated `docs/tui-boundary/touchpoints.md` containing:
  - Command list & grouping (list/start/stop/logs/preferences/telemetry/etc.).
  - Current transport/mechanism (direct RPC, file system, CLI helper).
  - Gaps / TODOs.
- Follow-up TODOs filed (or linked) for missing APIs.

## Exit Criteria
- [ ] Table/diagram added or updated.
- [ ] Gaps captured in roadmap or subsequent issues.
- [ ] Summary comment added to ADR-001 linking this audit.

## References
- ADR-001: TUI ↔ Core Interface Boundary
- Roadmap Issue: Phase P0
```

</details>

<details>
<summary><code>112-protocol-spec.md</code></summary>

```markdown
## Goal
- Propose and document the transport, schema, and versioning strategy for the UI ↔ core protocol.

## Deliverables
- ADR (or ADR update) containing:
  - Transport decision (HTTP/WebSocket/gRPC/etc.) with rationale.
  - Message schemas (request/response examples).
  - Error handling and status codes.
  - Version negotiation and compatibility plan.
- Diagram showing protocol layering vs existing coordinator APIs.

## Exit Criteria
- [ ] ADR merged and referenced by roadmap.
- [ ] Follow-up tasks identified (server implementation, client SDK).
- [ ] Reviewer sign-off from core + UI representatives.

## References
- ADR-001
- Issue #111 results
```

</details>

<details>
<summary>Templates for #113–#117</summary>

Create similar markdown files summarising goals, deliverables, and exit criteria for each phase.
</details>

