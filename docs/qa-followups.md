# QA Follow-ups

**Status**: Living document — update as items are completed or added.
**Last updated**: 2026-04-09
**Purpose**: single place to track everything the 2026-04-07/08/09 QA POC deferred, so nothing gets lost. Detailed specs live in the linked plan docs; this file is the index and status tracker.

Each item has:
- A short description
- Estimated effort
- Trigger (when to actually do it)
- Owner (who's responsible)
- Link to the detailed spec if one exists

---

## A. Quick wins (hours of work)

Small, self-contained items that can be picked up in any free slot.

### A-1. Wire adversarial LLM review into CI

- **Effort**: ~30 min (workflow addition) after the API key/secret is configured
- **Trigger**: once OpenAI Codex is set up (user mentioned this is pending)
- **Blocker**: `ANTHROPIC_API_KEY` (or equivalent Codex auth) as a GitHub Actions secret on `dora-rs/dora`
- **Owner**: heyong4725
- **Steps**:
  1. `gh secret set ANTHROPIC_API_KEY` or equivalent
  2. Add a workflow job to `.github/workflows/ci.yml` that runs `scripts/qa/adversarial.sh` on PRs and posts the output as a PR comment
  3. Tune the prompt template at `scripts/qa/adversarial-prompt.md` based on the first few runs
- **Reference**: `scripts/qa/adversarial.sh` (already works locally), `plan-agentic-qa-strategy.md` Section T1.4

### A-2. Architectural fitness tests

- **Effort**: ~1 hour per rule
- **Trigger**: whenever a cross-crate layering rule gets broken accidentally
- **Owner**: any contributor
- **Rules to encode** (sample list, not exhaustive):
  - `libraries/core` must not depend on `binaries/*`
  - `libraries/message` must not depend on `tokio` (protocol definitions should be runtime-agnostic)
  - Workspace must have exactly one `tokio` version across all crates
  - No crate except `shared-memory-server` may have more than 5 `unsafe` blocks
  - Each publishable crate must have a `README.md`
- **Implementation**: tests under `tests/architecture.rs` using `cargo_metadata`
- **Reference**: `plan-agentic-qa-strategy.md` Section 7.2 (T3.2)

### A-3. Expand miri coverage to more unsafe targets

- **Effort**: 1-2 hours per target
- **Trigger**: when writing tests for any pure-Rust unsafe code
- **Owner**: whoever touches the unsafe code next
- **Remaining targets** (miri-compatible, pure Rust):
  - `libraries/coordinator-store` (some unsafe in redb wrapper, pure Rust)
  - `libraries/arrow-convert` (currently 0 unsafe per audit; low priority)
  - `apis/rust/operator/types/src/lib.rs` (`slice::from_raw_parts` — needs focused unit tests that construct inputs without touching safer_ffi)
  - `apis/rust/node/src/event_stream/data_conversion.rs` — `Vec`-backed path (shmem-backed path can't be miri'd)
- **Do NOT target**: `shared-memory-server` (FFI via `shm_open`) — see §D-3 for the long-term fix
- **Pattern**: write focused unit tests first, then `cargo +nightly miri test -p <crate>`. See `metadata.rs` case study (commit `d12e6b8`) as the template
- **Reference**: `plan-agentic-qa-strategy.md` Section T2.3 (amended 2026-04-08)

### A-4. More mutation-score case studies

- **Effort**: 1-2 hours per file
- **Trigger**: when test coverage feels weak in a specific area, or as a focused audit
- **Owner**: any contributor
- **Top remaining targets** (by missed mutant count in last baseline):
  - `libraries/core/src/descriptor/validate.rs` — **69 missed** after the `parse_byte_size` / `parse_log_level` fixes. Next hotspots: `validate_ros2_qos` (9), `send_stdout_as` (8), `max_rotated_files` (7)
  - `libraries/coordinator-store/src/redb_store.rs` — ~40 missed after the boundary-test case study. Next focus: `list_daemons`, `get_daemon_by_machine`, and the parameter path beyond what the current 19 tests cover
  - `libraries/message/src/coordinator_to_cli.rs` — `Display` impls, `DataflowList::get_active`
- **Pattern**: see `4d2df85` (parse_byte_size) and `b878e50` (redb_store) as templates
- **Reference**: `qa-baseline-2026-04-07.md` for per-crate mutation scores

---

## B. Background investments (run, don't watch)

Expensive but unsupervised work. Kick off in a terminal, come back later.

### B-1. Full mutation re-run on daemon + coordinator with `test_workspace = true`

- **Effort**: ~8 hours background (daemon 5h + coordinator 3h)
- **Trigger**: before citing the daemon/coordinator mutation scores in any formal report
- **Owner**: heyong4725
- **Why**: the current 5.8% / 26.1% numbers are scoping artifacts, not real scores. `test_workspace = true` is now the default in `.cargo/mutants.toml` but the re-run hasn't been done
- **Command**: `cargo mutants --package dora-daemon --jobs 4 --timeout 120 --output /tmp/mutation/daemon-workspace` (similar for coordinator)
- **Expected outcome**: scores likely jump to 40-60%+ range based on the `fault_tolerance.rs` single-file experiment (21 missed → 21 caught)
- **Reference**: `.cargo/mutants.toml` comment block, commit `9e0c2c6`

### B-2. Unwrap ratchet ongoing cleanup

- **Effort**: incremental, 30 min here and there
- **Trigger**: whenever you're already in a file with unwraps
- **Owner**: any contributor
- **Current state**: 188 (down from 684 reported / 212 post-script-fix / 188 after tracing refactor)
- **Top offenders visible via**:
  ```bash
  while IFS= read -r file; do
    case "$file" in */tests.rs) continue ;; esac
    n=$(awk '/^[[:space:]]*#\[cfg\(test\)\]/ { exit }
             { c = gsub(/\.unwrap\(\)|\.expect\(/, "&"); if (c > 0) count += c }
             END { print count+0 }' "$file")
    [ "$n" -gt 0 ] && printf "%4d  %s\n" "$n" "$file"
  done < <(rg --files --type rust -g '!**/tests/**' -g '!**/benches/**' -g '!**/examples/**' -g '!**/build.rs' libraries/ binaries/ apis/) | sort -rn | head -10
  ```
- **Current top 5** (as of 2026-04-08):
  - `libraries/core/src/descriptor/visualize.rs`: 18 (all `writeln!` into String — legitimate infallible; do NOT refactor)
  - `binaries/cli/src/command/topic/echo.rs`: 17 (mostly `write!` / `serde_json::to_string` on infallible types)
  - `libraries/extensions/ros2-bridge/msg-gen/src/types/primitives.rs`: 11
  - `binaries/coordinator/src/prometheus_metrics.rs`: 10
  - `libraries/extensions/ros2-bridge/msg-gen/src/lib.rs`: 8
- **Target**: <150 by end of Q2 2026
- **Rule**: bump the budget down in the same commit that reduces the count

---

## C. Multi-day contractor work (specs complete, implementation pending)

Substantial work items. Each has a full spec ready to pick up.

### C-1. Fault injection scenarios

- **Effort**: 3-5 days of focused contractor work
- **Trigger**: tied to the Tier 2 rollout OR when investigating a specific reliability concern
- **Owner**: contractor assigned to QA
- **What**: 8 chaos scenarios — coordinator crash mid-transaction, daemon crash mid-replay, redb corruption, network partition, clock skew, disk full, OOM kill, SIGKILL during bulk writes
- **Prerequisites**: Option A scaffolding (subprocess-based test harness, ~100 lines)
- **Implementation priority**: start with scenario 3.3 (redb corruption — no subprocess needed) as the easiest first win
- **Reference**: **[`plan-fault-injection.md`](plan-fault-injection.md)** — complete operational spec with setup, invariants, and priority order

### C-2. Dogfood campaign first run

- **Effort**: ~1-2 days setup + 7 days background + 1 day post-analysis = ~10 calendar days
- **Trigger**: gated on the dora 1.0 release cycle per `plan-dora-1.0-consolidation.md`
- **Owner**: heyong4725 + contractor on-call
- **What**: run a realistic camera → vision_infer (Python) → detection_filter → logger/aggregator/metrics_sink workload across 2 machines for 168 hours, monitor 10 blocking success criteria
- **Prerequisites**:
  - Reference workload implementation under `scripts/dogfood/workload/` (not yet written)
  - Grafana dashboard JSON (not yet written)
  - Log aggregation (Loki or similar) deployed
- **Reference**: **[`plan-dogfood-campaign.md`](plan-dogfood-campaign.md)** — complete operational plan with pre-campaign checklist, kickoff commands, daily check cadence, failure handling, and artifact retention

### C-3. cargo-fuzz targets for protocol parsers

- **Effort**: 1-2 days initial + ongoing corpus maintenance
- **Trigger**: natural follow-up to the `ws_protocol` proptest work; the `Arbitrary` impls are the starting point
- **Owner**: contributor with async/fuzzing experience
- **Targets** (per the strategy doc):
  1. `fuzz/fuzz_targets/yaml_descriptor.rs` — `Descriptor::from_yaml_str`
  2. `fuzz/fuzz_targets/ws_protocol.rs` — `WsMessage` JSON deserialization
  3. `fuzz/fuzz_targets/topic_data_frame.rs` — WebSocket binary frame decode
  4. `fuzz/fuzz_targets/shared_memory_frame.rs` — shmem frame header parsing
  5. `fuzz/fuzz_targets/recording_format.rs` — `.adorec` reader
  6. `fuzz/fuzz_targets/arrow_ingest.rs` — Arrow RecordBatch from arbitrary bytes
  7. `fuzz/fuzz_targets/daemon_message.rs` — `DaemonToCoordinator` deserialize
- **Run cadence**: nightly with `cargo fuzz run <target> -- -max_total_time=600` per target
- **Corpus**: cache between runs via GitHub Actions cache; commit interesting findings to source control
- **Reference**: `plan-agentic-qa-strategy.md` Section T2.2

### C-4. Expanded E2E fault injection beyond the design doc

- **Effort**: ongoing as specific concerns surface
- **Trigger**: whenever a production-class bug is reported that the existing tests didn't catch
- **Owner**: whoever is debugging the reported issue
- **Practice**: every post-mortem should either add an E2E test that reproduces the failure OR add a fault injection scenario that matches it
- **Reference**: `plan-fault-injection.md` is the canonical place to add new scenarios

---

## D. Strategic / governance items

Not technical tasks — decisions and conversations that determine what "next" means.

### D-1. Governance conversation with phil-opp and haixuanTao

- **Effort**: one ~1-hour call + follow-up email
- **Trigger**: before any dora 1.0 consolidation work begins
- **Owner**: heyong4725
- **Critical path**: this is the gating decision for Phase -1 of `plan-dora-1.0-consolidation.md`. Until it happens, the whole consolidation plan is speculative
- **Agenda items**:
  1. Present the dora fork and the agentic engineering POC results (`qa-poc-report-2026-04-09.md` is the sharing doc)
  2. Present the consolidation proposal (`plan-dora-1.0-consolidation.md` — emphasize Phase 3b Zenoh SHM as the architectural anchor)
  3. Collect objections, concerns, and constraints
  4. Align on the critical-path Decision Points (D-1 through D-7 in the consolidation plan)
- **Reference**: [`plan-dora-1.0-consolidation.md`](plan-dora-1.0-consolidation.md) Section 5 Phase -1

### D-2. External security audit cadence

- **Effort**: 1-2 weeks per audit (outsourced)
- **Trigger**: before any minor-version release (0.3.0, 0.4.0, 1.0.0, ...)
- **Owner**: heyong4725 to commission; contractor/reviewer to execute
- **Cadence**: per minor version, rotating reviewers between rounds
- **Next run**: tied to dora 1.0 release per `plan-dora-1.0-consolidation.md` D-2
- **History**: `docs/audit-report-2026-03-21.md`, `docs/audit-report-2026-03-13.md`
- **Reference**: `plan-agentic-qa-strategy.md` Section T3.1, `plan-dora-1.0-consolidation.md` Decision Point D-2

### D-3. Zenoh SHM migration (sequencing decision)

- **Effort**: 2-3 days of implementation in Phase 3b of the dora 1.0 consolidation
- **Trigger**: during the dora 1.0 consolidation merge, not before and not after
- **Owner**: contractor executing Phase 3b
- **Why**: closes the `shared-memory-server` miri/proptest/fuzz blind spot by removing the code. Aligns with upstream dora (which already made the switch in `dora-rs/adora#1378`). Deletes ~660 lines and ~11 unsafe blocks. Eliminates the pinned `raw_sync_2 =0.1.5` fork dep
- **Sequencing options** (Decision Point D-7):
  - D-7a: Full migration in Phase 3b (~3 days extra on critical path)
  - D-7b: Defer to dora 1.1 (ships 1.0 with the known QA gap)
  - **D-7c (recommended)**: Data plane in Phase 3b, control channels deferred to 1.1
- **Reference**: [`plan-zenoh-shared-memory.md`](plan-zenoh-shared-memory.md), [`plan-dora-1.0-consolidation.md`](plan-dora-1.0-consolidation.md) Phase 3b + Decision Point D-7

### D-4. Fix the misleading `NodeId::TryFrom<String>` API (follow-up beyond the doc fix)

- **Effort**: ~1 day (requires a SemVer-breaking change)
- **Trigger**: next time `dora-message` has a breaking change anyway, or the dora 1.0 consolidation
- **Owner**: contributor with SemVer confidence
- **What**: the doc comment was fixed in `b878e50` to point users at `parse::<NodeId>()` instead of the misleading `TryFrom`. But the **real** fix is to override the blanket `TryFrom<String>` impl with an explicit one that returns `Result<Self, InvalidId>`. This is a breaking change (callers relying on the blanket Infallible type will break) so it's gated on a SemVer bump
- **Reference**: `libraries/message/src/id.rs` lines ~56-67, commit `b878e50`

### D-5. Consider removing `From<String>` panics entirely

- **Effort**: ~2 days across the workspace (needs audit of all call sites)
- **Trigger**: dora 1.0 API cleanup
- **Owner**: contributor doing the 1.0 API freeze
- **What**: `NodeId`, `OperatorId`, `DataId`, and similar ID types all have `From<String>` impls that panic on invalid input. This is a trap (see D-4). A clean 1.0 API would remove these impls entirely and force all call sites to use `parse::<_>()`. Mechanical but invasive
- **Reference**: `libraries/message/src/id.rs`

---

## E. Done but worth revisiting

Items that are "done enough" but could be improved if a specific problem surfaces.

| Item | Current state | What could improve it |
|---|---|---|
| Unwrap-budget script | Works correctly after 2026-04-08 fix | Replace grep/awk with `syn`-based parser for 100% accurate test-code exclusion |
| Coverage gate | Soft on PRs (70% on touched lines) | Raise to 80% once codebase catches up; pair with a branch-coverage gate (llvm-cov supports it but we don't use it yet) |
| Adversarial review prompt | Works for the types of bugs we've seen | Tune with more PR-level data once CI integration exists |
| `.cargo/mutants.toml` equivalent-mutant list | 1 entry (types_match) | Will grow as more mutation testing happens; each entry needs explicit reasoning |

---

## Quick-scan status table

| Item | Category | Effort | Blocker |
|---|---|---|---|
| A-1. Adversarial review in CI | Quick win | 30 min | Codex API key |
| A-2. Architectural fitness tests | Quick win | 1 hr / rule | None |
| A-3. Expand miri coverage | Quick win | 1-2 hrs / target | None |
| A-4. More mutation case studies | Quick win | 1-2 hrs / file | None |
| B-1. Full mutation re-run | Background | 8 hrs | None — just run it |
| B-2. Unwrap cleanup | Background | Incremental | None |
| C-1. Fault injection | Contractor | 3-5 days | Not a blocker, but best tied to a reliability concern |
| C-2. Dogfood campaign | Contractor | 10 calendar days | Tied to dora 1.0 release cycle |
| C-3. cargo-fuzz targets | Contractor | 1-2 days | None |
| C-4. E2E fault injection | Contractor | Ongoing | None |
| D-1. Governance conversation | Strategic | 1 hr | Scheduling |
| D-2. External audit cadence | Strategic | Per release | Funding / vendor selection |
| D-3. Zenoh SHM migration | Strategic | Part of consolidation | Gated on D-1 |
| D-4. NodeId TryFrom fix | Strategic | 1 day | Gated on a SemVer bump |
| D-5. Remove From<String> panics | Strategic | 2 days | Gated on 1.0 API freeze |

---

## Maintaining this document

When you complete an item:
1. Move it to `§E` (Done but worth revisiting) with a short note on what was done, OR
2. Delete it entirely if no follow-up remains

When a new item surfaces:
1. Add it to the appropriate section (A/B/C/D)
2. Fill in all the fields (effort, trigger, owner, reference)
3. Update the Quick-scan table at the bottom

When the context for an item changes (e.g., a blocker resolves, a spec gets written):
1. Update the trigger / blocker fields
2. Note the change date at the top

This document is the single source of truth for "what's still open on QA". If an item isn't here, it's not tracked.

---

## Related

- [`qa-runbook.md`](qa-runbook.md) — how to run existing QA day-to-day
- [`qa-poc-report-2026-04-09.md`](qa-poc-report-2026-04-09.md) — what was built, what was found, lessons learned
- [`plan-agentic-qa-strategy.md`](plan-agentic-qa-strategy.md) — the full strategy doc
- [`qa-baseline-2026-04-07.md`](qa-baseline-2026-04-07.md) — metrics baseline with case studies
