# Dora 1.0 Consolidation Plan

**Status**: Active — rename complete (PR #186), ready for upstream push
**Date**: 2026-04-10 (updated from 2026-04-07 draft)
**Author**: heyong4725 (with AI assistance)
**Scope**: This repo (`dora-rs/adora`) is the feature superset of upstream `dora-rs/dora`. The rename from `adora` → `dora` is complete. This plan describes pushing this repo's tree into `dora-rs/dora` as **dora 1.0.0**.

---

## 1. Executive Summary

This repo (`dora-rs/adora`) is a Rust-first fork of dora that has accumulated a significant superset of features while serving as a proof-of-concept for agentic AI engineering. Upstream dora (`dora-rs/dora`, 3,188 stars, 356 forks, at v0.5.0) has the brand equity, crates.io footprint, and community. Rather than maintaining two parallel projects, this plan pushes this repo's tree into `dora-rs/dora` as **dora 1.0.0**.

**Key update (2026-04-10):** The `adora` → `dora` rename is complete (PR #186). All crate names, types, env vars, YAML prefixes, CLI commands, and docs now use `dora` naming. No backward-compat aliases for the old `adora` names are kept — this repo has no external users depending on `Adora*` names. The consolidation is now a tree replacement into upstream, not a migration requiring compat shims.

**Consolidation rule:** where dora and dora overlap on source code and public APIs, dora's version wins. Where they diverge on tests, examples, or contributor history, the rule is **union**, not replacement. dora's git history is preserved as the first parent of the consolidation merge commit.

**Headline facts:**

| Metric | dora (upstream) | dora (this repo) |
|---|---|---|
| Version | 0.5.0 (2026-03-25) | 0.2.1 (2026-04-06) |
| GitHub stars | 3,188 | 4 |
| Forks | 356 | 9 |
| Open issues | 175 | (track separately) |
| Default branch | main | main |
| License | Apache-2.0 | Apache-2.0 |
| Top contributors | phil-opp (2012), haixuanTao (1873) | heyong4725 + agents |
| Language primary | Rust | Rust |
| Last push | 2026-04-07 (today, active) | 2026-04-07 |
| Workspace crates | ~30 | ~45 |
| Rust source files | n/a (not yet audited in upstream) | 252 |
| Cargo.toml files | n/a | 63 |

**Timeline estimate:** ~4 weeks end-to-end, gated on governance alignment with dora maintainers.

**Critical prerequisites before executing:**
1. Alignment with phil-opp and haixuanTao (top contributors) on the consolidation approach.
2. A dry-run wire-protocol compatibility audit (Phase -1 below).
3. A 1-week dogfood campaign on a real robotics workload on dora's current tree.
4. `Q1-minimum` test-infrastructure items landed (see `plan-agentic-qa-strategy.md`).

---

## 2. Goals and Non-Goals

### 2.1 Goals

- **Preserve dora's brand equity** — keep the `dora-rs/dora` repository URL, star count, issue tracker, external links, and crates.io namespace.
- **Ship dora's feature superset** as the canonical dora implementation.
- **Preserve contributor attribution** for both dora and dora 0.x history.
- **Provide a clean migration path** for existing dora 0.x users (migration guide + maintained legacy branch).
- **Frame the agentic engineering story honestly** as the launch narrative — not hide it.
- **Establish dora as an agentic-engineering-maintained project** going forward (see `plan-agentic-qa-strategy.md` for the quality framework that makes this responsible).

### 2.2 Non-goals

- **Not a line-by-line merge.** A three-way merge between two independently evolved trees produces garbage. This is a tree replacement with history preservation.
- **Not a silent rebrand.** Users and contributors are informed openly via blog post and migration guide.
- **Not a ROS1→ROS2 split.** We do not maintain two parallel projects long-term. Dora becomes dora 1.0 and the dora repo is archived with a pointer.
- **Not backwards-breaking where it can be avoided.** Upstream dora 0.x users get a migration guide and a maintained `v0.x` legacy branch. No compat shims for the internal `adora` → `dora` rename (no external adora users exist).

---

## 3. Current State: dora vs dora

### 3.1 Workspace layout diff

**Crates present in dora but not in dora** (dora additions — these become dora 1.0 additions):

| Path | Purpose |
|---|---|
| `apis/python/cli` | Python bindings for the CLI (`dora.build()`, `dora.run()`, etc.) |
| `apis/rust/compat/dora-node-api` | **Existing compat shim** — re-exports dora types under dora names. Will be **reversed** in 1.0 |
| `apis/rust/compat/dora-operator-api` | Same for operator API |
| `binaries/ros2-bridge-node` | Standalone ROS2 bridge node binary |
| `binaries/replay-node` | Replays `.adorec` recordings as dataflow sources |
| `binaries/record-node` | Records dataflow traffic to `.adorec` format |
| `libraries/log-utils` | Shared logging utilities |
| `libraries/coordinator-store` | Persistent state backend for coordinator (parameters, daemon state) |
| `libraries/recording` | `.adorec` format reader/writer |
| `libraries/shared-memory-server` | Zero-copy IPC, separated from communication-layer |
| `libraries/extensions/ros2-bridge/arrow` | Arrow-typed message conversion for ROS2 bridge |

**Example directories in dora not in dora**:

- `examples/ros2-bridge/yaml-bridge-service/{requester-node, handler-node}`
- `examples/ros2-bridge/yaml-bridge-action/goal-node`
- `examples/ros2-bridge/yaml-bridge-action-server/handler-node`
- `examples/service-example/{client, server}`
- `examples/action-example/{client, server}`
- `examples/log-sink-tcp`, `log-sink-file`, `log-sink-alert`
- `examples/cross-language/{rust-sender, rust-receiver}`
- `examples/validated-pipeline/{source, transform, sink}`

**Crates present in dora but not in dora** (dora removals — need explicit decision):

| Path | Status in dora | Action |
|---|---|---|
| `libraries/communication-layer/*` (request-reply) | Removed, replaced by service/action patterns in node API | Keep removed, document in migration guide as "replaced by `send_service_request()` helper" |
| `examples/error-propagation/{producer, consumer}` | Not in dora | Port to dora before merge — error propagation is a dora feature worth preserving |

**Files only present in dora docs or CI that dora should absorb:**

Run pre-merge:

```bash
git diff --name-status dora/main dora/main -- \
  | grep '^D' | awk '{print $2}' > dora-unique-files.txt
```

Expected categories: some CI workflows (cargo-release.yml, pip-release.yml, claude-code.yml, regenerate-schemas.yml, dora-bot-assign.yml, docker-image.yml, labeler.yml), README sponsorship/logo sections, CONTRIBUTING conventions, possibly some docs pages, possibly the `docker-image.yml` and `cargo-update.yml` automation.

### 3.2 Workspace-level dependency diff

| Dependency | dora 0.5.0 | dora 0.2.1 | Winner | Notes |
|---|---|---|---|---|
| arrow | 54.2.1 | 58 | **dora** | Major version jump; dora has already migrated callsites |
| pyo3 | 0.23 | 0.28 | **dora** | dora dropped `experimental-inspect` feature |
| pythonize | 0.23 | 0.28 | **dora** | Follows pyo3 |
| zenoh | 1.1.1 | ~1.8 | **dora** | Several releases; API-compatible within 1.x |
| serde_yaml | 0.9.33 | 0.9.33 | tie | |
| thiserror | (not pinned) | 2.0 | **dora** | Adopt in dora 1.0 |
| git2 | 0.20.4 | 0.20.4 | tie | |
| MSRV | 1.85.0 | 1.85.0 | tie | |
| edition | 2024 | 2024 | tie | |

**Risk flags:**
- arrow 54→58 is a multi-major jump with API changes. dora has migrated; downstream users still on dora 0.x will need to migrate their own arrow code. Migration guide must cover this.
- pyo3 0.23→0.28 will affect every downstream Python node that uses pyo3 directly. Migration guide must link to pyo3 changelog.
- zenoh 1.1→1.8 should be transparent to users but worth smoke-testing the distributed scenarios.

### 3.3 Protocol and wire-format diff

**Message crate (`libraries/message/src/`) — files in each:**

| File | dora | dora | Notes |
|---|---|---|---|
| `cli_to_coordinator.rs` | ✓ | ✓ | likely diverged — dora has WebSocket control plane |
| `common.rs` | ✓ | ✓ | |
| `config.rs` | ✓ | ✓ | |
| `coordinator_to_cli.rs` | ✓ | ✓ | |
| `coordinator_to_daemon.rs` | ✓ | ✓ | likely diverged — dora has state catch-up |
| `daemon_to_coordinator.rs` | ✓ | ✓ | |
| `daemon_to_daemon.rs` | ✓ | ✓ | |
| `daemon_to_node.rs` | ✓ | ✓ | likely diverged — dora has service/action metadata |
| `descriptor.rs` | ✓ | ✓ | YAML descriptor — likely diverged (new fields) |
| `id.rs` | ✓ | ✓ | |
| `integration_testing_format.rs` | ✓ | ✓ | |
| `lib.rs` | ✓ | ✓ | |
| `metadata.rs` | ✓ | ✓ | dora added metadata constants for service/action |
| `node_to_daemon.rs` | ✓ | ✓ | |
| **`auth.rs`** | ✗ | ✓ | **dora-only** — bearer token auth |
| **`ws_protocol.rs`** | ✗ | ✓ | **dora-only** — WebSocket control plane messages |

**Action:** a dedicated wire-protocol audit is needed before Phase 1 of the migration. See Section 7.1 "Decision Point D-1" below. Key question: if a dora 0.5.0 daemon and a dora 1.0.0 daemon end up in the same distributed deployment during a rolling upgrade, will they interoperate? If the answer is no, we need a 0.6.0 bridge release before 1.0.

### 3.4 CI and release pipeline diff

**dora `.github/workflows/`**: 11 files
- ci.yml, release.yml, cargo-release.yml, cargo-update.yml, pip-release.yml, claude-code.yml, docker-image.yml, dora-bot-assign.yml, labeler.yml, delete-buildjet-cache.yml, regenerate-schemas.yml

**dora `.github/workflows/`**: 3 files
- ci.yml, guide.yml, release.yml

**Gaps dora should absorb from dora:**
- `cargo-update.yml` — scheduled dependency updates (currently missing in dora)
- `docker-image.yml` — publishes a `dora:latest` docker image (worth keeping — downstream robotics deployments use it)
- `pip-release.yml` — separate PyPI release (dora merges into release.yml; worth evaluating which pattern is cleaner)
- `regenerate-schemas.yml` — auto-regenerates YAML schemas from Rust structs (highly useful for agentic engineering, should keep)
- `labeler.yml` — PR auto-labeling
- `dora-bot-assign.yml` — PR auto-assignment

**Decision:** port dora's missing workflows to dora's tree before merge. Treat them as bug fixes that dora missed while diverging.

### 3.5 Documentation diff

**dora `docs/` inventory** (34 files):
- API refs: `api-rust.md`, `api-python.md`, `api-c.md`, `api-cxx.md`
- Architecture: `architecture.md` (860 lines)
- Plans: `plan-dynamic-topology.md`, `plan-soft-realtime.md`, `plan-coordinator-ha.md`, `plan-zenoh-shared-memory.md`, `plan-event-loop-separation.md`, `plan-arrow-ipc-framing.md`
- Protocol: `websocket-control-plane.md`, `websocket-topic-data-channel.md`
- User guides: `quickstart.md`, `cli.md`, `debugging.md`, `python-guide.md`, `yaml-spec.md`, `types.md`, `patterns.md`
- Ops: `distributed-deployment.md`, `fault-tolerance.md`, `performance.md`, `realtime-tuning.md`, `logging.md`, `modules.md`
- ROS2: `ros2-bridge.md`
- Audits: `audit-report-2026-03-13.md`, `audit-report-2026-03-21.md`
- Integration: `octos-dora-integration-report.md`
- Compat: `dora-compatibility.md`
- Sprint: `sprint-plan-v0.1.1.md`, `roadmap-v0.2.md`
- Testing: `testing-guide.md` (432 lines)

**dora `docs/` inventory**: limited to `docs/src/` directory (smaller than dora's docs). Most dora documentation lives on `dora-rs.ai` website rather than in-repo.

**Action for docs:**
- Use dora's docs as the base for dora 1.0 (already superset).
- Audit dora's website (`dora-rs.ai`) for pages that aren't reflected in dora's in-repo docs — rewrite or link as needed.
- Update all internal doc links from `dora` → `dora` during the rename pass.
- Remove `octos-dora-integration-report.md`, `sprint-plan-v0.1.1.md`, and `roadmap-v0.2.md` as dora-specific artifacts (archive to `docs/archive/` or drop).
- Consolidate the two audit reports into one `docs/audit-history.md` with timestamped sections.
- Rename `dora-compatibility.md` → `migration-from-0.x.md` and invert its direction.

### 3.6 Known technical debt in dora (per 2026-03-21 audit)

From `docs/audit-report-2026-03-21.md`:

- **3 memory-safety issues**: shmem OOB read, unsound `Sync` impl, ARM data race
- **1 code execution vulnerability**: URL downloads without integrity verification
- **30+ correctness bugs** across daemon, coordinator, APIs
- **Performance roadmap items** from 2026-03-13 audit still unaddressed
- **Smoke/E2E tests not in CI** at time of audit (partially addressed since — see `tests/example-smoke.rs` wiring)

**Before the 1.0 release:** all memory-safety and code-execution issues must be verified fixed or have an explicit waiver documented. The 1.0 launch cannot ship with unresolved criticals from the audit. See Decision Point D-2 below.

---

## 4. Overlap Resolution Matrix

This is the canonical source of truth for "what wins" when dora and dora diverge.

| Axis | Rule | Rationale | Exceptions |
|---|---|---|---|
| Source tree (`*.rs` files) | **dora wins wholesale** | Superset; cleaner to replace than merge | Where dora has a bug fix dora never received, cherry-pick the fix onto the consolidated tree |
| Crate names | **dora-* wins** | Brand reclaim is the point of consolidation | dora-* crates published as deprecated re-export shims for 6 months |
| Binary name | **dora wins** | Same | Ship `dora` as a deprecated symlink in the dora deb/wheel for 1 minor version |
| File formats (`.adorec` recording, YAML) | **dora wins**, but **dora formats remain readable** | Zero-day breakage for existing dora users is unacceptable | Daemon should read both `.adorec` and old dora recording formats; `dora migrate-yaml` command for config upgrades |
| Public Rust APIs | **dora wins** | Superset | Every removed/renamed API documented in migration guide with a replacement |
| Python APIs | **dora wins** | Superset | Same |
| C / C++ APIs | **dora wins** | Superset | Already has macro-based compat layer in dora (`node_api.h` → see `docs/dora-compatibility.md`) |
| Wire protocol | **dora wins**, but **see Decision Point D-1** | Needs dedicated audit | May require a 0.9 bridge release that speaks both |
| Tests | **Union** | dora tests encode years of user-reported bug signal | If a dora test fails on dora tree, fix dora's implementation, do not delete the test |
| Examples | **Union, dora-superset** | dora examples are tutorials users may have bookmarked | Port dora examples that don't exist in dora; update dora-branded examples in place |
| Documentation | **dora wins as base, dora-rs.ai pages backfilled** | dora docs are more comprehensive | Dora website references dora APIs that may differ; update both |
| Git history | **dora tree, union history** | Preserve contributor attribution | See Appendix A for the merge recipe |
| CI workflows | **Union, dora augmented with dora's missing jobs** | dora has release automation dora lacks | See 3.4 above |
| Release cadence | **dora's release.yml as base**, reintegrate dora's cargo-release.yml pattern | dora has better release automation in some areas | Harmonize the two |
| Issue tracker | **dora's, with dora issues ported** | dora-rs/dora repo is the destination | Script the port; close dora issues with links |
| Stars / watchers / forks | **dora's, unchanged** | They live on the dora repo by definition | — |

**Corollary rule:** if any row in this table is contested during the consolidation, the default is to **defer the decision** and add it to Section 9 "Open Questions" for maintainer discussion. No unilateral calls on contested overlaps.

---

## 5. Migration Plan: Phases

### Phase -1: Governance and prerequisites (1 week)

Before any code moves, resolve the governance layer.

**Tasks:**
1. **Maintainer conversation.** Schedule a call with phil-opp and haixuanTao (top dora contributors). Present this document. Collect their objections, concerns, and constraints. Document outcomes.
2. **Freeze decision timing.** Agree on a date range when dora main is frozen for the merge (no competing merges during Phase 1).
3. **Contractor alignment.** Brief the contractor maintainers on the plan. Collect any local knowledge about dora features dora may not know about.
4. **Protocol audit.** Run the wire-protocol comparison (see Appendix E audit commands). Determine whether a rolling upgrade from dora 0.5.0 to dora 1.0.0 is possible. If not, decide whether to:
   - (a) Ship dora 0.9.0 as a bridge release speaking both protocols, then 1.0.0 cleanly, or
   - (b) Ship 1.0.0 directly and document "full restart required" as a release note.
5. **Audit debt closure.** Verify all critical items from `docs/audit-report-2026-03-21.md` are fixed. Block 1.0 on any unresolved memory-safety or code-execution issues.
6. **Downstream user list.** GitHub code search for `dora-node-api` imports. List top 10 projects by stars. Reach out to maintainers with early access to the migration guide.
7. **PyPI ownership verification.** Confirm `dora-rs` package on PyPI is owned by the same entity. Confirm `dora-rs` ownership.
8. **crates.io ownership verification.** `cargo owner --list dora-node-api`, etc. Confirm all `dora-*` and `dora-*` crate names are under the same account.

**Gate:** Phase -1 is complete when:
- [ ] Maintainers have signed off in writing (GitHub issue, email, Slack, whatever is your contractor norm).
- [ ] Protocol audit output is attached to this plan doc as Appendix F.
- [ ] All audit-2026-03-21 criticals are closed or waived.
- [ ] PyPI and crates.io ownership is verified.
- [ ] 1-week dogfood campaign on current dora main has started (runs in background through Phases 0-3).

### Phase 0: Preparation (2 days)

**Tasks:**
1. In `dora-rs/dora`: create tag `v0.x-final` on current main. Push.
2. In `dora-rs/dora`: create long-lived branch `legacy/0.x` from the tag. Push.
3. In `dora-rs/dora`: create tag `dora-0.2.1-baseline` on current main. Push.
4. In `dora-rs/dora`: freeze main branch (branch protection blocks pushes). Post pinned issue explaining the freeze.
5. Open a draft PR `dora-rs/adora#XXX: Draft: dora 1.0 consolidation` against main. This is where Phase 1-5 commits land.
6. Set up a clean working checkout of `dora-rs/dora` locally for the contractor or agent doing the merge.
7. Add `dora` as a remote in that checkout.

### Phase 1: Mechanical merge (1 day)

See Appendix A for the exact git recipe.

**Tasks:**
1. From `dora-rs/dora`, create branch `v1.0-rewrite` from main.
2. `git fetch dora main`.
3. Use `git merge -s ours --allow-unrelated-histories dora/main` followed by `git read-tree -u --reset dora/main` and `git commit --amend` to produce a merge commit whose tree is identical to dora's but whose history includes dora 0.x as an ancestor.
4. Verify: `git diff dora/main` is empty and `git log --graph` shows both histories joining.
5. Commit message: "merge: adopt dora as dora 1.0 baseline" (see Appendix A for full text).
6. Push `v1.0-rewrite` to dora.

**Gate:** `cargo check --all --exclude <python packages>` builds at least as well as dora main did at the time of the baseline tag.

### Phase 2: Rename pass (2-3 days) — COMPLETE

**Done in PR #186.** 675 files changed. All `adora` → `dora` across crate names, types, env vars, YAML prefixes, CLI, docs, schemas, recording format. No compat aliases kept.

**Tasks:**
1. **Crate name rename**: in every `Cargo.toml`, change `dora-*` → `dora-*` in `[package].name`, `[dependencies]`, and `[workspace.dependencies]`. Use the script in Appendix B.
2. **Binary name rename**: `binaries/cli/Cargo.toml` `[[bin]].name = "dora"`. Update install scripts, CI `cp target/release/dora /usr/local/bin/dora` → `dora`.
3. **Source imports**: `dora_node_api` → `dora_node_api`, etc. in every `.rs` file. Same script.
4. **Public identifiers**: `DoraNode` → `DoraNode` (and keep `DoraNode` as a deprecated type alias for 1 version). `DoraStatus` → `DoraStatus`. `DoraOperator` → `DoraOperator`. See Appendix B for full list.
5. **Macro names**: `dora::register_operator!` → `dora::register_operator!`.
6. **Docstrings and comments**: context-aware — do not sed these. Hand-review or use a scoped find-and-review tool.
7. **Docs**: rewrite all `dora` references in `docs/` prose. Rename `dora-compatibility.md` → `migration-from-0.x.md` with inverted direction.
8. **Recording format**: `.adorec` → `.drec`. Magic bytes: `DORAREC\x00` / `DORAEND\x00`. Clean break.
9. **Config file paths**: `~/.config/adora/` → `~/.config/dora/`. Clean break.
10. **YAML descriptor keywords**: `adora/timer/...` → `dora/timer/...`. Clean break — all examples updated.
11. **C/C++ headers**: `ADORA_STATUS_*` macros removed. `DORA_STATUS_*` is canonical (from auto-generated `operator_types.h`).
12. **Python package**: `import dora` is canonical. No `import adora` shim.
13. **README**: rewrite. Lead with dora's identity, mention the rewrite story, credit contributors.
14. **CHANGELOG**: new `1.0.0` section. Include a "Breaking changes" subsection with a table.

**Verification loop:**
```bash
cargo check --all --exclude <python>   # fix each error
cargo clippy --all -- -D warnings
cargo fmt --all -- --check
cargo test --all --exclude <python>
./scripts/smoke-all.sh --rust-only
```

Iterate until all four are green.

**Gate:** `v1.0-rewrite` branch builds, lints clean, all tests pass, smoke tests pass.

### Phase 3: Test and example union (3-5 days)

**Tasks:**
1. Identify dora-only tests:
   ```bash
   git diff v0.x-final v1.0-rewrite -- 'tests/' 'libraries/*/src/**/*.rs' \
     --name-only --diff-filter=D > /tmp/dora-tests-lost.txt
   ```
2. For each file in `/tmp/dora-tests-lost.txt`:
   - If it's a test and the feature still exists in dora tree: port the test.
   - If it tests a feature dora removed: document in migration guide under "Removed features"; do not port.
   - If it's a regression test for a bug: verify the bug is not reintroduced in dora's tree; port or add a new test.
3. Identify dora-only examples:
   ```bash
   git diff v0.x-final v1.0-rewrite -- 'examples/' --name-only --diff-filter=D
   ```
4. Port dora examples that don't exist in dora. Specifically: `examples/error-propagation/`, any `cmake-dataflow` artifacts dora dropped, any benchmark variants.
5. Port dora CI workflows that dora lacks (see 3.4 above): `cargo-update.yml`, `docker-image.yml`, `regenerate-schemas.yml`, `labeler.yml`, `dora-bot-assign.yml`, `delete-buildjet-cache.yml`.
6. Port dora docs pages that dora-rs.ai uses but aren't in dora's `docs/`. Do a manual audit of the website.
7. Re-run the verification loop from Phase 2.

**Gate:** dora 1.0 tree contains a strict union of dora and dora test coverage; verification loop still passes.

### Phase 3b: Adopt Zenoh shared memory (2-3 days)

**Added 2026-04-08 after the Q1 POC on dora exposed `shared-memory-server` as a QA blind spot.** See `plan-agentic-qa-strategy.md` section 10a.4 and `plan-zenoh-shared-memory.md`.

**Rationale.** Dora's `libraries/shared-memory-server` crate has 30 `unsafe` blocks handling every local node↔daemon message (4 control channels per node + data regions ≥ 4KB). It is the largest concentration of unsafe code in the workspace and cannot be analyzed by miri (FFI), property testing (no pure-Rust entry points), or fuzzing. Upstream dora already migrated to Zenoh's native SHM in `dora-rs/adora#1378`, which deletes ~660 lines of DropToken lifecycle code, removes ~11 unsafe blocks from `channel.rs`, gives 35% lower latency on messages ≥ 64B and 3-10× throughput on messages ≥ 2KB, and removes the pinned `raw_sync_2 =0.1.5` fork dependency.

**Doing this as part of the consolidation, not before or after, is the right sequencing:**
- **Before the consolidation**: migration would touch code that is about to be replaced — wasted work, double-tested.
- **After the consolidation**: the 30 unsafe blocks ship in dora 1.0 under the dora brand, inheriting the QA gap dora already has.
- **During the consolidation**: the migration naturally aligns dora's tree with upstream dora, removes code that was never going to be QA-covered, and closes the miri gap by removing the uncoverable code entirely. The consolidation is already editing these files, so the churn is already paid for.

**Tasks:**
1. **Port `dora-rs/adora#1378`** into the `v1.0-rewrite` branch. This is the source code change dora has been queueing.
2. **Delete** `apis/rust/node/src/node/drop_stream.rs`, `DropToken` types in `libraries/message/src/common.rs`, `pending_drop_tokens` and `check_drop_token` in daemon, and the 4 blocking shmem threads per node.
3. **Rewrite** `apis/rust/node/src/node/mod.rs` to add a zenoh session + `ShmProvider` to `DoraNode`.
4. **Update** `send_output` to publish via zenoh when data ≥ threshold.
5. **Update** `apis/rust/node/src/event_stream/data_conversion.rs` to add `RawData::ZenohShm(ZShm)`.
6. **Update** `libraries/core/src/topics.rs` with zenoh topic helpers.
7. **Add** recording support by copying `ZShm` data at the record-node level (the PR skips this; we must restore it because dora has `.adorec` support that upstream doesn't).
8. **Keep the 4KB threshold** for small messages — they continue via the existing TCP control path (per `plan-zenoh-shared-memory.md` recommendation).
9. **Handle memlock gracefully** with a fallback + warning for `dora run` local dev.
10. **Pre-warm the zenoh session** during node init to avoid the 16× first-message latency spike.

**Dependencies to remove during this phase:**
- `shared_memory_extended` (niche, low maintenance)
- `raw_sync_2 =0.1.5` (exact-version pin on a custom fork)

**Dependencies to verify:**
- `zenoh` must stay pinned to `~1.8` (already done in dora)
- `zenoh`'s `shared-memory` + `unstable` features must be enabled

**Gate:** `cargo test --all --exclude <python>` passes on the rewrite branch; benchmark regression shows p50 latency improved vs dora 0.x; the `shared-memory-server` crate is either deleted or reduced to <5 unsafe blocks; migration guide documents the breaking change for custom node implementations whose data path consumed `Option<Arc<DataMessage>>`.

**Risk:** zenoh's SHM API is marked `unstable`. It has been stable since zenoh 0.10 (2023) but could change in zenoh 2.0. Mitigation: pinned to `~1.8`, monitor zenoh releases during the 1.0 post-release period.

**Follow-up (post-1.0, possibly 1.1):** Phase 3 of `plan-zenoh-shared-memory.md` — migrate the 4 control channels to zenoh as well. This fully deletes `shared-memory-server` and eliminates all custom shmem code from the workspace.

### Phase 4: Compat layer (2 days)

**Tasks:**
1. **Invert the compat direction.** dora currently has `apis/rust/compat/dora-node-api` that re-exports dora. In 1.0, this becomes `apis/rust/compat/dora-node-api` that re-exports dora. Rename directories and swap re-exports. Add `#[deprecated]` attribute with migration note.
2. **Final dora-* shim crates for crates.io.** Publish `dora-cli 0.3.0`, `dora-node-api 0.3.0`, `dora-operator-api 0.3.0`, etc. as thin wrappers around `dora-* 1.0.0`. Each crate's `lib.rs`:
   ```rust
   #![deprecated(
       since = "0.3.0",
       note = "dora has been consolidated into dora 1.0. \
               Use `dora-node-api` instead. See migration guide: \
               https://dora-rs.ai/docs/migration-from-dora"
   )]
   pub use dora_node_api::*;
   ```
3. **PyPI shim**: `dora-rs 0.3.0` depends on `dora-rs>=1.0.0`, re-exports `from dora import *` in its `__init__.py`, emits a `DeprecationWarning` on import. Maintain for 6 months.
4. **dora 0.x to 1.0 compat shims** (the more important direction):
   - For each dora 0.x API that changed in 1.0, provide a `dora::compat::v0` module with the old signature marked `#[deprecated]`, wrapping the new implementation.
   - Start with the top 10 most-used dora 0.x APIs (determined by downstream-user grep).
   - Remove in 1.1.0.
5. **YAML descriptor compat**: daemon reads dora 0.x YAML with deprecation warnings; `dora migrate-yaml <file>` command upgrades in place.
6. **Write a `dora migrate` subcommand** that upgrades a project directory from dora 0.x: updates `Cargo.toml` dependencies, renames YAML fields, prints a report of manual steps required.

**Gate:** a sample dora 0.x project (pick one from the downstream user list) builds and runs on dora 1.0 with only automated `dora migrate` changes plus the documented manual steps.

### Phase 5: Release (2 days)

**Tasks:**
1. Final verification: `cargo test --all`, full E2E, benchmark regression, smoke tests, cargo-audit clean, coverage baseline reported.
2. Dogfood campaign evidence attached to the release PR. If dogfood found issues, fix them first.
3. Merge `v1.0-rewrite` → `main` on dora repo via `--no-ff` merge to preserve the phased history.
4. Tag `v1.0.0` on the merge commit. Annotated, signed if your release norm is to sign.
5. Push tag. Release workflow fires:
   - Builds cross-platform CLI binaries.
   - Publishes `dora-* 1.0.0` to crates.io (in dependency order).
   - Publishes `dora-rs 1.0.0` to PyPI.
   - Creates GitHub release with release notes.
6. After `dora-* 1.0.0` is live on crates.io:
   - Publish the deprecated `dora-* 0.3.0` shims pointing at dora.
   - Publish `dora-rs 0.3.0` PyPI shim.
7. **Announcement:**
   - Blog post on `dora-rs.ai`: "dora 1.0: A Rust-First Rewrite, Built with Agentic Engineering."
   - Post to HN, `/r/rust`, `/r/robotics`, Rust Weekly, Discord.
   - Email the downstream user list with a direct link to the migration guide.
8. **Archive `dora-rs/dora` repository** via GitHub Settings. Pin a README pointing at `dora-rs/dora`. Do not delete the repo — external links may exist.

**Gate:** `dora 1.0.0` is publicly released, announcement is live, archive is done.

### Phase 6: Post-release support (ongoing for 6 weeks)

**Tasks:**
1. Monitor the GitHub issue tracker for migration pain points. Fast follow-ups on any reported regressions.
2. Maintain `legacy/0.x` branch for critical security fixes only. Backport policy: CVEs only, nothing else.
3. Publish `dora 1.0.1`, `1.0.2` as needed within the first month.
4. Update blog comments, HN thread, Reddit thread with answers to common questions.
5. Kick off **Q1 full rollout** (see `plan-agentic-qa-strategy.md`). The 1.0 release is the end of the Q1-minimum subset and the beginning of the Tier 1+2+3 investment.

---

## 6. Risk Register

| ID | Risk | Likelihood | Impact | Mitigation |
|---|---|---|---|---|
| R-1 | Wire protocol incompatibility breaks rolling upgrades | Medium | High | Phase -1 protocol audit; ship 0.9 bridge if needed |
| R-2 | Downstream users break on crate rename | High | High | 6-month deprecation shim crates; migration guide; dora-rs/dora/issues/downstream-impact tracking issue |
| R-3 | Hidden dora bug surfaces in production under dora brand | Medium | High | Q1-minimum test infrastructure must land before 1.0; dogfood campaign; adversarial review |
| R-4 | Contributor backlash from rewrite framing | Low | Medium | Blog post credits original contributors by name; merge commit includes both histories; honest framing of agentic engineering as experiment |
| R-5 | Maintainers (phil-opp, haixuanTao) object to consolidation | Medium | Critical | Phase -1 governance conversation; collect objections; be willing to adjust the plan or walk away |
| R-6 | crates.io name collision or yank issues | Low | Medium | Verify ownership in Phase -1; publish in strict dependency order |
| R-7 | PyPI package name conflict | Low | Medium | Verify ownership in Phase -1 |
| R-8 | Arrow 54→58 or pyo3 0.23→0.28 breaks downstream Python nodes | High | Medium | Migration guide links upstream changelogs; dora migrate subcommand flags this |
| R-9 | Audit critical issues resurface under dora brand | Low (if Phase -1 gate enforced) | Critical | Phase -1 gates on audit closure; independent audit pre-release |
| R-10 | ROS2 bridge users lose functionality | Medium | High | ROS2 bridge superset claim verified in Phase -1; audit dora-unique bridge features |
| R-11 | GitHub issue tracker mass-reports post-release | Medium | Medium | Dedicated post-release triage rotation for 2 weeks; canned responses for common migration questions |
| R-12 | Blog post gets dunked on for "AI rewrote it and broke my stuff" | Medium | Medium | Honest framing; lead with test/verification story not velocity story |
| R-13 | Legacy 0.x branch rots and users on 0.x stop getting security fixes | High (over 6+ months) | Medium | Define explicit EOL date in the migration guide; give 6 months notice |
| R-14 | Merge commit confuses `git blame` or breaks third-party tooling | Low | Low | Documented in README; `--first-parent` works |
| R-15 | Zenoh SHM migration (Phase 3b) regresses latency on small messages | Medium | Medium | Keep 4KB threshold; messages below it continue via existing TCP path; benchmark regression gate blocks release if p99 drops |
| R-16 | Zenoh SHM `unstable` API breaks in a future zenoh release | Low (6-12 months) | Medium | Pin to `~1.8`; monitor zenoh releases; CI nightly rebuild against zenoh's main branch as early-warning |
| R-17 | Zenoh SHM recording (`.adorec`) support not finished before release | Medium | High | Phase 3b explicitly includes recording support via a ZShm → Vec<u8> copy at record-node level; gate the release on `examples/validated-pipeline` recording end-to-end test passing |

---

## 7. Decision Points (for maintainer discussion)

These are the decisions that cannot be made unilaterally. Each needs explicit resolution before the relevant phase begins.

### D-1: Wire protocol compatibility strategy

**Context:** dora added `auth.rs` and `ws_protocol.rs` to `libraries/message/`. Several of dora's existing message files have likely diverged. A dora 0.5.0 daemon may not interoperate with a dora 1.0.0 daemon.

**Options:**
- **D-1a:** Ship 1.0.0 with a hard protocol break. Document "full cluster restart required for upgrade." Simple.
- **D-1b:** Ship 0.9.0 as a bridge release with both protocols; 1.0.0 drops the 0.x protocol. Slower but allows rolling upgrades.
- **D-1c:** Ship 1.0.0 with a version-negotiation handshake; daemons auto-downgrade when talking to 0.x peers. Most work; best UX.

**Recommendation:** D-1a if the Phase -1 protocol audit shows the divergence is small and rolling upgrades are rare in the dora user base; D-1b otherwise. Avoid D-1c unless a specific user demands it.

**Decision owner:** maintainers + top 3 downstream users

### D-2: Audit critical closure evidence

**Context:** audit-2026-03-21 flagged 3 memory-safety issues, 1 code-execution vulnerability, and 30+ correctness bugs. Some are fixed, some are not. The 1.0 release cannot ship with any of the first four unresolved.

**Options:**
- **D-2a:** Formal re-audit by the same reviewer before 1.0, producing an explicit "all criticals resolved" letter.
- **D-2b:** Contractor-maintained checklist walked through line-by-line, with links to fix commits.
- **D-2c:** New independent audit (different firm/person) as a clean-slate assessment.

**Recommendation:** D-2c for credibility with the dora user base. A 1.0 launch with a line like "independently audited by X" is worth the cost.

**Decision owner:** heyong4725

### D-3: Removed features policy

**Context:** dora removed `libraries/communication-layer/*` (dora's request-reply). A few dora features may have been removed that still have active users.

**Options:**
- **D-3a:** Document removed features in migration guide; users on those features stay on 0.x.
- **D-3b:** Cherry-pick removed features back into 1.0 as deprecated modules.
- **D-3c:** For each removed feature, decide individually based on usage.

**Recommendation:** D-3c. Most removals are probably correct, but some are not. Run the downstream-usage grep in Phase -1 to identify which features have users.

**Decision owner:** maintainers

### D-4: Legacy 0.x branch EOL

**Context:** `legacy/0.x` branch needs a defined support policy. Supporting two branches indefinitely is not sustainable.

**Options:**
- **D-4a:** 6 months of CVE-only fixes, then EOL.
- **D-4b:** 12 months of CVE-only fixes.
- **D-4c:** No support — 0.x is immediately EOL at 1.0 release.

**Recommendation:** D-4a. Enough for users to migrate, not so long that it becomes a maintenance burden. EOL date goes in the migration guide.

**Decision owner:** maintainers

### D-5: agentic engineering disclosure in the release

**Context:** dora 1.0 is the first AI-engineered major release. This is either a selling point or a liability depending on framing.

**Options:**
- **D-5a:** Lead with the story — blog post headline mentions agentic engineering explicitly.
- **D-5b:** Mention it in the post but not the headline — lead with features.
- **D-5c:** Don't mention it publicly; treat it as an implementation detail.

**Recommendation:** D-5a, with strong emphasis on the verification layer (Q1 testing plan). "We rewrote dora with agents and here is how we verified it's safe to ship" is a more honest and interesting narrative than hiding the process. The Q1 plan is the proof.

**Decision owner:** heyong4725 + marketing/community lead

### D-7: Zenoh SHM migration scope and timing

**Context:** Added 2026-04-08 after the Q1 POC on dora exposed `shared-memory-server` as the largest concentration of unsafe code in the workspace (30 blocks) and a QA blind spot (can't be analyzed by miri, proptest, or fuzz because tests call libc's `shm_open`). Upstream dora already adopted Zenoh SHM in `dora-rs/adora#1378`. Phase 3b of this plan proposes folding the migration into the consolidation merge.

**Options:**
- **D-7a:** Do it in Phase 3b as currently written. Deletes ~660 lines, removes ~11 unsafe blocks, eliminates `raw_sync_2` pinned fork dep, aligns with upstream. Adds 2-3 days to the critical path.
- **D-7b:** Defer to dora 1.1. Consolidation ships with the current shared-memory-server; zenoh migration happens as a follow-up. Shorter 1.0 critical path; 1.0 ships with the known QA gap.
- **D-7c:** Partial: migrate the data plane in Phase 3b (Phase 1 of `plan-zenoh-shared-memory.md`) but keep the 4 control channels on custom shmem until 1.1. This is roughly what upstream dora did.

**Recommendation:** D-7c. Full migration is ~2-3 days more work than partial, and the control channels are relatively stable. D-7a risks scope creep; D-7b ships the QA gap under the dora brand.

**Decision owner:** maintainers (phil-opp, haixuanTao) + heyong4725

### D-6: Archive or maintain dora repo

**Context:** post-release, what happens to `dora-rs/dora`?

**Options:**
- **D-6a:** Archive with pinned README pointing at dora.
- **D-6b:** Delete (not recommended — breaks external links).
- **D-6c:** Keep as a future-experiments sandbox separate from dora's stable release.

**Recommendation:** D-6a. If you want a future-experiments sandbox, create `dora-rs/dora-next` or similar; don't conflate it with the old dora repo.

**Decision owner:** heyong4725

---

## 8. Rollback Plan

If the consolidation encounters a blocking issue during Phase 1-4, rollback is cheap because the dora repo's main branch is untouched until the merge in Phase 5.

**If the issue is found in Phase 1-4:**
- Close the draft PR.
- Keep `v1.0-rewrite` branch for reference.
- Return to dora main as the active development branch.
- Retry when the blocker is resolved.

**If the issue is found after Phase 5 merge but before the v1.0.0 tag:**
- Revert the merge commit on dora main.
- Fix on `v1.0-rewrite` branch.
- Re-merge.

**If the issue is found after v1.0.0 is tagged but before crates are published:**
- Delete the tag (rare but recoverable).
- Fix on a new `v1.0.1-fix` branch.
- Re-tag and release as v1.0.1.

**If the issue is found after crates.io publish:**
- Yank the affected crate versions (`cargo yank`).
- Publish v1.0.1 with the fix.
- Do not unpublish — yank leaves existing users' lockfiles working but prevents new pulls.

**If the issue is catastrophic** (e.g., published crates contain a backdoor):
- Yank all crates.
- Post a GitHub Security Advisory.
- Pull the GitHub Release.
- Emergency blog post.
- This is the single scenario where contractors should page heyong4725 immediately.

**Legacy branch fallback:** the `legacy/0.x` branch and `v0.x-final` tag remain untouched by any of the above. Users can always pin to 0.x as their escape hatch.

---

## 9. Open Questions

These are questions this plan cannot answer without maintainer input or investigation. Each should be resolved before Phase -1 completes.

1. Does dora 0.5.0 have any feature that dora removed that still has active users? (Need downstream audit.)
2. Is the ROS2 bridge in dora a strict superset of dora's, or are there dora-only type coverages, QoS levels, or convenience features? (See `docs/ros2-bridge.md` and memory file `ros2-bridge-gaps.md`.)
3. Does dora have CI/release secrets (PyPI tokens, crates.io tokens, GitHub deploy keys, cache credentials) that dora's release.yml is not configured for? (Check `.github/workflows/*.yml` env refs and repo secrets.)
4. Is the `dora-rs` PyPI package name owned by the same entity as `dora-rs/dora` GitHub org? Are there any legal/account issues with transitioning control?
5. Do any downstream projects depend on dora's git tags or commit SHAs rather than crates.io versions? If so, the tag renaming strategy matters.
6. Is `dora-bot-assign.yml` configured to use a GitHub App or a personal token? If the latter, who's token? Does it need to be rotated during the transition?
7. Does the `dora-rs.ai` documentation site pull from the repo's `docs/src/` or from a separate documentation repo? Does dora's docs structure match what the site expects?
8. Are there any dora contributors who have signed a CLA that dora contributors have not? License-wise both are Apache-2.0, but CLA footprints can differ.
9. Is there a dora Discord / Slack with users who should be informed ahead of the public release?
10. Who currently has admin rights to the `dora-rs` GitHub org? Is the consolidation approach acceptable to all of them, not just the contractor maintainers?

---

## 10. Timeline Estimate

| Phase | Description | Duration | Dependencies |
|---|---|---|---|
| -1 | Governance, prereqs, protocol audit | 1 week | — |
| 0 | Preparation (tags, branches, freeze) | 2 days | Phase -1 complete |
| 1 | Mechanical merge | 1 day | Phase 0 complete |
| 2 | Rename pass | 2-3 days | Phase 1 green |
| 3 | Test and example union | 3-5 days | Phase 2 green |
| 3b | Zenoh SHM migration (data plane, D-7c) | 2-3 days | Phase 3 green |
| 4 | Compat layer | 2 days | Phase 3b green |
| 5 | Release | 2 days | Phase 4 green + dogfood evidence |
| 6 | Post-release support | 6 weeks (ongoing) | Phase 5 complete |

**Critical path:** ~4-5 weeks from Phase -1 kickoff to 1.0 tag, assuming no blockers in governance or protocol audit. Phase 3b adds 2-3 days vs. the original estimate.

**Parallel tracks during Phases 1-4:**
- Dogfood campaign (1 week of real-workload running)
- Q1-minimum test infrastructure (coverage baseline, cargo-audit, see QA plan)
- Blog post and migration guide drafting
- Downstream user outreach

---

## 11. Success Criteria

**Technical:**
- [ ] `dora 1.0.0` is tagged and released on `dora-rs/dora`.
- [ ] All `dora-*` crates published on crates.io with 1.0.0 version.
- [ ] `dora-rs` published on PyPI with 1.0.0 version.
- [ ] `dora-*` shim crates published as deprecated with 0.3.0 version.
- [ ] `dora-rs` shim published as deprecated on PyPI.
- [ ] `dora-rs/dora` repository archived with pinned README.
- [ ] `legacy/0.x` branch exists on `dora-rs/dora` and CI runs on it.
- [ ] Migration guide published at `dora-rs.ai/docs/migration-from-0.x`.
- [ ] Coverage baseline, cargo-audit clean, and dogfood evidence published as part of the release notes.
- [ ] At least 3 downstream projects have confirmed successful migration to 1.0 via `dora migrate`.

**Narrative:**
- [ ] Blog post published and received without significant backlash.
- [ ] Original dora contributors credited by name.
- [ ] Agentic engineering story framed as experiment + verification, not as velocity flex.
- [ ] HN / Reddit / Discord discussions are net-positive (subjective; target: >70% positive sentiment).

**Community:**
- [ ] No regression in dora star growth rate (monitor 30 days post-release).
- [ ] Issue tracker has <20% "migration pain" issues in the first 30 days.
- [ ] No contributor publicly objects to the consolidation process.

---

## 12. Appendix A: Git recipe for the mechanical merge

```bash
# --- Setup ---
cd ~/src/dora-rs-dora  # clean checkout of dora-rs/dora
git remote add dora https://github.com/dora-rs/adora.git
git fetch dora main

# --- Preserve dora 0.x ---
git checkout main
git pull origin main
git tag -a v0.x-final -m "Final commit of dora 0.x series before 1.0 rewrite baseline"
git branch legacy/0.x main
git push origin v0.x-final legacy/0.x

# --- Create the rewrite branch starting from dora's tree ---
git checkout -b v1.0-rewrite dora/main

# --- Record dora 0.x as a parent via "ours" merge ---
# -s ours says: merge but keep current tree (dora's, since we're on dora-based branch).
# --allow-unrelated-histories is required because dora 0.x and dora have diverged enough
# that git doesn't auto-detect the common ancestor.
git merge -s ours --allow-unrelated-histories origin/main \
  -m "merge: adopt dora as dora 1.0 baseline

dora is a Rust-first rewrite of dora built with agentic AI engineering
as a proof-of-concept over Q1 2026. This commit consolidates dora's
tree back into dora as the basis for dora 1.0.0.

Dora's source tree is taken wholesale. dora 0.x history is preserved
via this commit's first parent and is reachable as the 'legacy/0.x'
branch and 'v0.x-final' tag.

All dora 0.x contributors remain attributed in git history; run
  git shortlog -sne HEAD
to see the combined contributor list.

Consolidation plan: docs/plan-dora-1.0-consolidation.md
Migration guide: docs/migration-from-0.x.md

Co-Authored-By: phil-opp <...>  # fill in
Co-Authored-By: haixuanTao <...>
# ... add all primary contributors
"

# --- Verify ---
git diff dora/main  # should be empty; tree matches dora wholesale
git log --graph --oneline -30  # should show the merge joining both histories
git log --first-parent --oneline | head -20  # should show dora's linear timeline

# --- Push ---
git push -u origin v1.0-rewrite
```

**Important caveats:**
- The `--allow-unrelated-histories` flag is necessary because, while dora is technically a fork of dora, the histories have diverged enough and been re-versioned (`v0.2.1` vs `v0.5.0`) that git considers them effectively unrelated for merge purposes.
- `-s ours` keeps the **current branch's tree**. Since we checked out `dora/main`, "current" means dora. This is the intended direction.
- After the merge commit, `git blame` will attribute each line to whichever side (dora or dora) last touched it in its native history — which in practice means dora, because dora's tree won.
- `git shortlog -sne HEAD` will show both dora 0.x contributors and dora contributors combined, with their respective commit counts.

---

## 13. Appendix B: Crate and identifier rename map

### Crate names

| dora name | dora 1.0 name |
|---|---|
| `dora-cli` | `dora-cli` |
| `dora-daemon` | `dora-daemon` |
| `dora-coordinator` | `dora-coordinator` |
| `dora-runtime` | `dora-runtime` |
| `dora-core` | `dora-core` |
| `dora-message` | `dora-message` |
| `dora-arrow-convert` | `dora-arrow-convert` |
| `dora-download` | `dora-download` |
| `dora-tracing` | `dora-tracing` |
| `dora-metrics` | `dora-metrics` |
| `dora-log-utils` | `dora-log-utils` |
| `dora-coordinator-store` | `dora-coordinator-store` |
| `dora-recording` | `dora-recording` |
| `shared-memory-server` | `dora-shared-memory-server` (namespace for crates.io) |
| `dora-node-api` | `dora-node-api` |
| `dora-node-api-c` | `dora-node-api-c` |
| `dora-node-api-python` | `dora-node-api-python` |
| `dora-operator-api` | `dora-operator-api` |
| `dora-operator-api-c` | `dora-operator-api-c` |
| `dora-operator-api-python` | `dora-operator-api-python` |
| `dora-operator-api-macros` | `dora-operator-api-macros` |
| `dora-operator-api-types` | `dora-operator-api-types` |
| `dora-ros2-bridge` | `dora-ros2-bridge` |
| `dora-ros2-bridge-msg-gen` | `dora-ros2-bridge-msg-gen` |
| `dora-ros2-bridge-python` | `dora-ros2-bridge-python` |
| `dora-cli-api-python` | `dora-cli-api-python` |
| `dora-examples` | `dora-examples` (not published) |

### Rust identifiers (public)

| dora identifier | dora 1.0 identifier | Compat |
|---|---|---|
| `DoraNode` | `DoraNode` | `type DoraNode = DoraNode` (deprecated 1 version) |
| `DoraStatus` | `DoraStatus` | `type DoraStatus = DoraStatus` |
| `DoraOperator` | `DoraOperator` | `trait` re-export |
| `DoraOutputSender` | `DoraOutputSender` | type alias |
| `dora_core::*` | `dora_core::*` | module re-export |
| `DoraError` (if present) | `DoraError` | type alias |
| `DoraEvent` | `DoraEvent` | type alias |

### C / C++ identifiers

Inverse of the current `node_api.h` / `operator_api.h` macros. Before 1.0, these files have `#define dora_foo dora_foo`. After 1.0, they have `#define dora_foo dora_foo` with a deprecation comment.

### Python identifiers

| dora | dora 1.0 |
|---|---|
| `import dora` | `import dora` (canonical) |
| `from dora import Node` | `from dora import Node` |
| `dora.DoraStatus` | `dora.DoraStatus` |
| `dora.build()` | `dora.build()` |
| `dora.run()` | `dora.run()` |
| `dora.testing.MockNode` | `dora.testing.MockNode` |
| `dora.builder.DataflowBuilder` | `dora.builder.DataflowBuilder` |
| `dora-rs` (PyPI) | `dora-rs` (PyPI) |

### File extensions and config paths

| Before (adora) | dora 1.0 | Compat |
|---|---|---|
| `.adorec` (recording) | `.drec` | Clean break — no `.adorec` files exist in the wild |
| `~/.config/adora/` | `~/.config/dora/` | Clean break — internal only |
| `ADORA_*` env vars | `DORA_*` env vars | Clean break — no external users |
| `adora/timer/...` YAML | `dora/timer/...` YAML | Clean break — all examples updated |

### Script for mechanical rename

```bash
#!/bin/bash
# rename-dora-to-dora.sh — run from repo root after Phase 1 merge

set -euo pipefail

# 1. Cargo.toml package names
find . -name 'Cargo.toml' -not -path '*/target/*' -exec sed -i.bak \
  -e 's/^name = "dora-/name = "dora-/' \
  -e 's/^name = "shared-memory-server"$/name = "dora-shared-memory-server"/' \
  {} \;

# 2. Cargo.toml dependencies (both quoted keys and bare keys)
find . -name 'Cargo.toml' -not -path '*/target/*' -exec sed -i.bak \
  -e 's/dora-\([a-z][a-z-]*\) = /dora-\1 = /g' \
  -e 's/"dora-\([a-z][a-z-]*\)"/"dora-\1"/g' \
  {} \;

# 3. Rust source imports and identifiers
find . -name '*.rs' -not -path '*/target/*' -exec sed -i.bak \
  -e 's/\bdora_core\b/dora_core/g' \
  -e 's/\bdora_node_api\b/dora_node_api/g' \
  -e 's/\bdora_operator_api\b/dora_operator_api/g' \
  -e 's/\bdora_message\b/dora_message/g' \
  -e 's/\bdora_arrow_convert\b/dora_arrow_convert/g' \
  -e 's/\bdora_tracing\b/dora_tracing/g' \
  -e 's/\bdora_metrics\b/dora_metrics/g' \
  -e 's/\bdora_download\b/dora_download/g' \
  -e 's/\bdora_daemon\b/dora_daemon/g' \
  -e 's/\bdora_coordinator\b/dora_coordinator/g' \
  -e 's/\bdora_runtime\b/dora_runtime/g' \
  -e 's/\bdora_cli\b/dora_cli/g' \
  -e 's/\bdora_recording\b/dora_recording/g' \
  -e 's/\bdora_ros2_bridge\b/dora_ros2_bridge/g' \
  {} \;

# 4. Public type renames — careful, these are word-boundary
find . -name '*.rs' -not -path '*/target/*' -exec sed -i.bak \
  -e 's/\bDoraNode\b/DoraNode/g' \
  -e 's/\bDoraStatus\b/DoraStatus/g' \
  -e 's/\bDoraOperator\b/DoraOperator/g' \
  -e 's/\bDoraOutputSender\b/DoraOutputSender/g' \
  -e 's/\bDoraError\b/DoraError/g' \
  -e 's/\bDoraEvent\b/DoraEvent/g' \
  {} \;

# 5. Clean up backups
find . -name '*.bak' -not -path '*/target/*' -delete

# 6. Verify
cargo check --all --exclude dora-node-api-python --exclude dora-operator-api-python --exclude dora-ros2-bridge-python
```

**Run checklist after this script:**
1. `cargo check` — iterate until green.
2. `cargo clippy --all -- -D warnings` — iterate.
3. `cargo fmt --all` — format.
4. `cargo test --all` — iterate.
5. `rg '\bdora' --type rust` — should only match in compat shim re-exports, test fixtures, or commit messages quoted in docs. Hand-review each remaining hit.
6. `rg '\bDora' --type rust` — same.
7. Review all `*.md` files for `dora` in prose. Hand-edit.
8. Review `*.yml` CI files for `dora` references (install paths, cache keys).

---

## 14. Appendix C: Downstream user impact checklist

Before Phase -1 completes, produce this list:

1. GitHub code search: `https://github.com/search?q=dora-node-api+language%3ARust&type=code`
2. For each result:
   - Repo name
   - Maintainer GitHub handle
   - Stars
   - Last update
   - Whether they pin to a specific dora version
3. Rank by stars × recency.
4. Take the top 10.
5. For each top-10 project:
   - File a courtesy GitHub issue on their repo titled "Heads up: dora 1.0 is coming — we'd love your feedback on the migration guide"
   - Include a link to this plan doc and the migration guide draft.
   - Ask: "Would you be willing to test `dora migrate` on your project before the 1.0 release?"
6. Collect responses. Adjust the migration guide and `dora migrate` subcommand based on feedback.

**This step is not optional.** The single biggest reputational risk of the consolidation is surprise breakage at the downstream user level. Early outreach turns potential critics into collaborators.

---

## 15. Appendix D: Dogfood campaign plan

Run in parallel with Phases 1-4. Purpose: generate evidence for the 1.0 release claim.

**Workload:** pick a real robotics scenario that exercises the full stack. Recommended:
- Rust sensor node producing Arrow-formatted camera frames at 30 Hz.
- Python operator running a simple vision model (e.g., YOLO inference).
- Rust sink logging structured outputs.
- Distributed: 2 daemons across 2 machines.
- Duration: 7 days continuous.

**Measurements:**
- Total uptime
- Number of unexpected restarts (target: 0)
- Number of shared-memory OOM events (target: 0)
- Number of dropped messages (target: 0)
- p50 / p99 / p99.9 latency (for release notes)
- Memory growth over 7 days (target: flat after warmup)
- CPU utilization

**Outputs:**
- A Grafana dashboard snapshot for each day.
- A `docs/dogfood-evidence-v1.0.md` doc with the dataset.
- A one-paragraph "what we found" section in the blog post.

**Gate:** if any target is missed, fix and re-run. No 1.0 release without a clean dogfood week.

---

## 16. Appendix E: Audit commands for Phase -1

Run these to collect the data needed to complete this plan.

```bash
# Protocol audit: diff each message file between dora and dora
cd ~/src/dora-rs/dora
git remote add dora https://github.com/dora-rs/adora.git 2>/dev/null || true
git fetch dora main

for f in cli_to_coordinator.rs common.rs config.rs coordinator_to_cli.rs \
         coordinator_to_daemon.rs daemon_to_coordinator.rs daemon_to_daemon.rs \
         daemon_to_node.rs descriptor.rs id.rs metadata.rs node_to_daemon.rs; do
  echo "=== $f ==="
  git diff dora/main..HEAD -- "libraries/message/src/$f" | wc -l
done

# Public API diff using cargo-public-api
cargo install cargo-public-api
cd ~/src/dora-rs/dora
cargo public-api -p dora-node-api > /tmp/dora-node-api.txt
cd ~/src/dora-rs/dora  # clean checkout
cargo public-api -p dora-node-api > /tmp/dora-node-api.txt
diff /tmp/dora-node-api.txt /tmp/dora-node-api.txt > /tmp/api-diff.txt

# Downstream user grep
gh search code 'dora-node-api' --language rust --limit 50 --json repository \
  | jq '.[] | .repository.nameWithOwner' | sort -u > /tmp/downstream-users.txt

# Crate ownership verification
for crate in dora-node-api dora-operator-api dora-cli dora-core dora-message; do
  echo "=== $crate ==="
  cargo owner --list "$crate" 2>&1 || echo "  not published"
done

# PyPI ownership
pip index versions dora-rs 2>/dev/null || echo "dora-rs not on PyPI or index disabled"
pip index versions dora-rs 2>/dev/null || echo "dora-rs not on PyPI or index disabled"

# GitHub org admin list
gh api orgs/dora-rs/members --jq '.[] | .login'

# Test file counts
find ~/src/dora-rs/dora -name '*.rs' -path '*/tests/*' | wc -l
find ~/src/dora-rs/dora -name '*.rs' -path '*/tests/*' | wc -l

# Cargo.toml counts
find ~/src/dora-rs/dora -name Cargo.toml | wc -l
find ~/src/dora-rs/dora -name Cargo.toml | wc -l
```

Save the outputs as `docs/phase--1-audit-YYYY-MM-DD.md` and attach to this plan.

---

## 17. Appendix F: Protocol audit results

**(To be filled in during Phase -1.)**

Template:

```
## Protocol audit — <date>

### Scope
- dora version examined: 0.5.0 (<sha>)
- dora version examined: 0.2.1 (<sha>)

### Message-by-message diff

| File | Lines changed | Semantic breaks | Wire-compatible? |
|---|---|---|---|
| cli_to_coordinator.rs | N | describe | Yes/No |
| coordinator_to_daemon.rs | N | describe | Yes/No |
| daemon_to_node.rs | N | describe | Yes/No |
| ws_protocol.rs | (dora-only) | n/a | No |
| auth.rs | (dora-only) | n/a | No |
| ... | | | |

### Handshake compatibility
- Does dora 0.5.0 daemon recognize dora 1.0.0 coordinator? (Yes/No/Partial)
- Does version negotiation exist? (Yes/No)

### Recommendation
- D-1a / D-1b / D-1c (see Decision Points)
- Rationale: ...
```

---

## 18. Change log for this document

| Date | Author | Change |
|---|---|---|
| 2026-04-07 | heyong4725 + AI | Initial draft |

---

## 19. Related documents

- [`plan-agentic-qa-strategy.md`](plan-agentic-qa-strategy.md) — Quality and testing strategy that backs this consolidation. **Must-read companion.**
- [`dora-compatibility.md`](dora-compatibility.md) — Existing dora→dora compat layer documentation. Will be renamed and inverted as part of Phase 2.
- [`audit-report-2026-03-21.md`](audit-report-2026-03-21.md) — Dora technical debt inventory. Criticals must be closed before 1.0.
- [`architecture.md`](architecture.md) — System architecture reference.
- [`testing-guide.md`](testing-guide.md) — Current test infrastructure.
