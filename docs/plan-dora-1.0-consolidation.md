# Dora 1.0 Consolidation Plan

**Status** (2026-04-17): **Phases 0, 1, 3 landed on `dora-rs/dora` `v1.0-rewrite`. Phase 3b deferred into RC-window landings.** D-0 = D-0a (tree takeover), D-1 = D-1a (hard protocol break), D-7 = D-7c (partial Zenoh SHM) — landing progressively in the RC window as separate `rc.N` tags per #313 + #314, not pre-rc.1. No `dora migrate` command per #297. Release flow: **5a tag rc.1 → 5b iterate rc.N (rc.2 = adopt upstream's control-channel cleanup #313; rc.3 = zenoh SHM data-plane migration #314) → 5c tag GA with both landed**. See execution epic #305.
**Date**: 2026-04-16 (updated from 2026-04-10)
**Author**: heyong4725 (with AI assistance)
**Scope**: This repo (`dora-rs/adora`) is the feature superset of upstream `dora-rs/dora`. The rename from `adora` → `dora` is complete. This plan describes pushing this repo's tree into `dora-rs/dora` as **dora 1.0.0**.

---

## 1. Executive Summary

This repo (`dora-rs/adora`) is a Rust-first fork of dora that has accumulated a significant superset of features while serving as a proof-of-concept for agentic AI engineering. Upstream dora (`dora-rs/dora`, 3,188 stars, 356 forks, at v0.5.0) has the brand equity, crates.io footprint, and community. Rather than maintaining two parallel projects, this plan pushes this repo's tree into `dora-rs/dora` as **dora 1.0.0**.

**Key update (2026-04-10):** The `adora` → `dora` rename is complete (PR #186). All crate names, types, env vars, YAML prefixes, CLI commands, and docs now use `dora` naming. No backward-compat aliases for the old `adora` names are kept — this repo has no external users depending on `Adora*` names. The consolidation is now a tree replacement into upstream, not a migration requiring compat shims.

**Consolidation rule:** where dora and dora overlap on source code and public APIs, dora's version wins. Where they diverge on tests, examples, or contributor history, the rule is **union**, not replacement. dora's git history is preserved as the first parent of the consolidation merge commit.

**Headline facts:**

| Metric | `dora-upstream` (0.5.0) | `dora-fork` (this repo, 0.2.1) |
|---|---|---|
| Version | 0.5.0 (2026-03-25) | 0.2.1 (2026-04-06) |
| GitHub stars | 3,188 | 4 |
| Forks | 356 | 9 |
| Open issues | 175 | (track separately) |
| Default branch | main | main |
| License | Apache-2.0 | Apache-2.0 |
| Top contributors (GitHub "contributions" metric¹) | phil-opp 2,023 · haixuanTao 1,895 | heyong4725 + agents |
| Language primary | Rust | Rust |
| Last push | 2026-04-07 (active) | 2026-04-16 |
| Workspace crates | ~30 | ~45 |
| Rust source files | n/a (not yet audited in upstream) | 252 |
| Cargo.toml files | n/a | 63 |

> ¹ Upstream numbers are from `GET /repos/dora-rs/dora/contributors` on 2026-04-16 — GitHub's "contributions" counter, which measures commits on the default branch after PR merge (accounting for squash/rebase attribution). This is **not** the same as `git shortlog -sne` on a local clone, which counts raw commit authorships. On the fork today, `git shortlog -sne` reports Philipp Oppermann 1,786 and the three `haixuanTao` aliases summing to 1,828 — lower than the upstream contributions metric because the fork is behind upstream's latest commits and because the two metrics disagree systematically. §19.5 discusses contributor preservation in detail (issue #214); the headline table above uses the upstream-authoritative number for the upstream column.

**Timeline estimate:** ~4 weeks end-to-end, gated on governance alignment with dora maintainers.

**Critical prerequisites before executing:**
1. Alignment with phil-opp and haixuanTao (top contributors) on the consolidation approach.
2. A dry-run wire-protocol compatibility audit (Phase -1 below).
3. A 1-week dogfood campaign on a real robotics workload on dora's current tree.
4. `Q1-minimum` test-infrastructure items landed (see `plan-agentic-qa-strategy.md`).

### 1.1 Terminology (read first)

Because PR #186 renamed this repo `adora` → `dora`, both trees now literally share the string "dora". To keep the rest of this document unambiguous:

- **`dora-upstream`** — the canonical `dora-rs/dora` repository on GitHub, currently at v0.5.0 (3,188 stars). The consolidation **target**.
- **`dora-fork`** — this repository (`dora-rs/adora` on GitHub, v0.2.1). The consolidation **source**. Formerly named adora; the rename is substantively complete (crate names, types, CLI, env vars) but some trailing prose, URLs, issue-refs, and the `.adorec` file format still carry the old name. Finishing that pass is a pre-merge item (see §19.8).
- **`dora 1.0`** — the post-consolidation release published from `dora-upstream` after the merge.

Where older sections still say "dora" ambiguously, the rule is: if the sentence is describing the destination repo / brand equity / crates.io namespace, it means `dora-upstream`; if it's describing a tree, feature, or crate that comes from this repo, it means `dora-fork`. A full prose relabeling pass is pending (see P0-1 in the review PR).

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

## 3. Current State: `dora-upstream` vs `dora-fork`

> **Note (2026-04-16):** §19 ("Fresh Superset Audit") supersedes parts of this section with fresher numbers. Where §3 and §19 disagree, §19 is authoritative. A full merge of §19 back into §3 is a pending cleanup (see P1-7 in the review PR).

### 3.1 Workspace layout diff

**Crates present in `dora-fork` but not in `dora-upstream`** (fork additions — these become dora 1.0 additions):

| Path | Purpose |
|---|---|
| `apis/python/cli` | Python bindings for the CLI (`dora.build()`, `dora.run()`, etc.) |

> **Correction (2026-04-16 review):** earlier drafts listed `apis/rust/compat/dora-node-api` and `.../dora-operator-api` as "existing compat shims that will be reversed in 1.0". Neither directory exists today — not in `dora-fork` and not in `dora-upstream` (verified against `dora-rs/dora@main`). The Phase 4 compat layer is **new code to write**, not a directory rename. Phase 4 has been re-scoped accordingly.

| `binaries/ros2-bridge-node` | Standalone ROS2 bridge node binary |
| `binaries/replay-node` | Replays `.drec` recordings as dataflow sources |
| `binaries/record-node` | Records dataflow traffic to `.drec` format |
| `libraries/log-utils` | Shared logging utilities |
| `libraries/coordinator-store` | Persistent state backend for coordinator (parameters, daemon state) |
| `libraries/recording` | `.drec` format reader/writer |
| `libraries/shared-memory-server` | Zero-copy IPC, separated from communication-layer |
| `libraries/extensions/ros2-bridge/arrow` | Arrow-typed message conversion for ROS2 bridge |

**Example directories in `dora-fork` not in `dora-upstream`**:

- `examples/ros2-bridge/yaml-bridge-service/{requester-node, handler-node}`
- `examples/ros2-bridge/yaml-bridge-action/goal-node`
- `examples/ros2-bridge/yaml-bridge-action-server/handler-node`
- `examples/service-example/{client, server}`
- `examples/action-example/{client, server}`
- `examples/log-sink-tcp`, `log-sink-file`, `log-sink-alert`
- `examples/cross-language/{rust-sender, rust-receiver}`
- `examples/validated-pipeline/{source, transform, sink}`

**Crates present in `dora-upstream` but not in `dora-fork`** (fork removals — need explicit decision):

| Path | Status in `dora-fork` | Action |
|---|---|---|
| `libraries/communication-layer/*` (request-reply) | Removed, replaced by service/action patterns in node API | Keep removed, document in migration guide as "replaced by `send_service_request()` helper" |
| `examples/error-propagation/{producer, consumer}` | Not in fork | Port to `dora-fork` before merge — error propagation is an upstream feature worth preserving |

**Files only present in `dora-upstream` docs or CI that `dora-fork` should absorb:**

Run pre-merge:

```bash
# Assumes `upstream` = dora-rs/dora and `origin` = this repo.
git diff --name-status upstream/main origin/main -- \
  | grep '^D' | awk '{print $2}' > upstream-unique-files.txt
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

> **Update (2026-04-16):** this section was written when `dora-fork` had 3 workflows. It now has 14. The "gaps" list below is almost entirely closed — verify before blocking on it.

**`dora-upstream` `.github/workflows/`** (as of 2026-03-25): 11 files
- ci.yml, release.yml, cargo-release.yml, cargo-update.yml, pip-release.yml, claude-code.yml, docker-image.yml, dora-bot-assign.yml, labeler.yml, delete-buildjet-cache.yml, regenerate-schemas.yml

**`dora-fork` `.github/workflows/`** (as of 2026-04-16): 14 files
- ci.yml, guide.yml, release.yml, cargo-release.yml, cargo-update.yml, pip-release.yml, claude-code.yml, docker-image.yml, dora-bot-assign.yml, labeler.yml, delete-buildjet-cache.yml, regenerate-schemas.yml, nightly.yml, waiting-for-author.yml

**Gaps `dora-fork` had and has since closed:**
- `cargo-update.yml`, `docker-image.yml`, `pip-release.yml`, `regenerate-schemas.yml`, `labeler.yml`, `dora-bot-assign.yml`, `delete-buildjet-cache.yml` — all present.

**Workflows unique to `dora-fork`** (candidates to keep into 1.0):
- `nightly.yml` — nightly regression suite
- `waiting-for-author.yml` — stale-PR housekeeping
- `guide.yml` — mdBook guide build

**Decision:** re-diff against `dora-upstream/main` immediately before Phase 0 to capture any upstream workflow changes since 2026-03-25. The original "port upstream's missing workflows to fork" directive is effectively complete.

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
- **Smoke/E2E tests not in CI** at time of audit — addressed since. `tests/example-smoke.rs`, `tests/ws-cli-e2e.rs`, and `tests/fault-tolerance-e2e.rs` all run in CI (see `.github/workflows/ci.yml`).

**Before the 1.0 release:** all memory-safety and code-execution issues must be verified fixed or have an explicit waiver documented. The 1.0 launch cannot ship with unresolved criticals from the audit. See Decision Point D-2 below.

**Status (2026-04-16):** closure record [`docs/audit-2026-03-21-closure.md`](audit-2026-03-21-closure.md) walks every Critical finding. 17 fixed (with per-finding fix commits + current-HEAD verification), 3 eliminated by architectural change (DC3/CM3 → Phase 3b), 3 explicitly waived with rationale (CM1 analyzed correct; SEC1 hash opt-in with warning; SEC3 TLS deferred post-1.0). Every row has a decision recorded. Closed #289.

---

## 4. Overlap Resolution Matrix

This is the canonical source of truth for "what wins" when dora and dora diverge.

| Axis | Rule | Rationale | Exceptions |
|---|---|---|---|
| Source tree (`*.rs` files) | **dora wins wholesale** | Superset; cleaner to replace than merge | Where dora has a bug fix dora never received, cherry-pick the fix onto the consolidated tree |
| Crate names | **dora-* wins** | Brand reclaim is the point of consolidation | No crate-name transition required: upstream already used `dora-*` names on crates.io; fork never published `adora-*` crates (verified 2026-04-17). Only PyPI transition needed — `adora-rs 0.3.0` deprecated shim ships at Phase 5c pointing at `dora-rs 1.0.0`. |
| Binary name | **dora wins** | Same | Ship `dora` as a deprecated symlink in the dora deb/wheel for 1 minor version |
| File formats (`.drec` recording, YAML) | **dora wins**; 1.0 is a hard break (#297) | No production users identified (`docs/downstream-user-assessment-2026-04-16.md`); cost of backward-compat format readers + `migrate-yaml` tooling outweighs value | Manual rename `capture.adorec → capture.drec`; no format-compat shim, no `dora migrate-yaml` command; migration guide documents manual YAML field updates |
| Public Rust APIs | **dora wins** | Superset | Every removed/renamed API documented in migration guide with a replacement |
| Python APIs | **dora wins** | Superset | Same |
| C / C++ APIs | **dora wins** | Superset | Already has macro-based compat layer in dora (`node_api.h` → see `docs/migration-from-0.x.md`) |
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

### Phase 3b: Deferred — architectural cleanup moves to RC window (0 days)

> **Rescoped 2026-04-17 after verification (#309 closure comment).** The original D-7c plan assumed upstream had already migrated to zenoh SHM via PR `#1378`. Verification showed:
> - Upstream PR #1378 is **open, not merged**. 3,000 lines, Copilot-authored, mergeable=false. Experimental.
> - Upstream main doesn't use zenoh SHM at all. Upstream commit `01995ad67` (2026-03-18) *removed* the shmem control channel entirely (it was disabled-by-default) and kept `shared_memory_extended` for the data plane.
> - Forcing the zenoh migration in before rc.1 bundles high-risk code with the mechanical merge, muddies regression attribution, and inherits unreviewed upstream work.
>
> **Revised approach:** architectural changes (control-channel cleanup + zenoh SHM migration) land **progressively during the Phase 5b RC window** as separate `rc.N` tags with independent dogfood. 1.0 GA ships with both.

**Phase 3b is now a placeholder.** No code work happens here. Proceed directly from Phase 3 (#308, closed) to Phase 4 (#310).

Landing sequence:
- **rc.1** — fork's current architecture (includes custom shmem control channel, disabled by default).
- **rc.2** — Option D: adopt upstream's [`01995ad67`](https://github.com/dora-rs/dora/commit/01995ad67) equivalent. Remove shmem control-channel wiring, keep data-plane shmem (`shared_memory_extended` via `shared-memory-server` re-exports). Tracked in #313.
- **rc.3** — Zenoh SHM data-plane migration. Port or improve upstream PR #1378 based on its state at work-start. Includes fork-specific additions (`.drec` recording with ZShm, memlock fallback, session pre-warm). Tracked in #314.
- **rc.N** — each architectural landing gets its own 7-day clean dogfood before moving forward.
- **GA (rc.N → v1.0.0)** — ships with both D and zenoh SHM.

**Why this is safer than the original D-7c timing:**
- rc.1 provides a stable-known-architecture reference for attribution.
- D and zenoh land separately — regression signals are cleanly identifiable.
- Upstream PR #1378 may merge during this window; if so, we port the reviewed version rather than the Copilot draft.
- Plan §7 D-7 outcome effectively remains D-7c (partial migration lands in 1.0), just with different timing and a safer gate sequence.

**1.1 follow-up** (unchanged from original D-7c rationale): migrate the 4 local control channels to zenoh too; delete `shared-memory-server` entirely; drop `raw_sync_2` pinned fork dep; close the miri coverage gap fully. Tracked in `plan-zenoh-shared-memory.md` Phase 3.

### Phase 4: Compat layer (0.5 day)

> **Scope correction (2026-04-17 review):** the original plan assumed three compat surfaces were needed — Rust crate-name shims on crates.io, a `dora::compat::v0` Rust module, and a PyPI shim. Re-verification against published registries and the fork's actual public API cut the scope to one real item.
>
> - **No Rust crate-name shims needed.** `adora-*` crates were never published to crates.io (verified 2026-04-17 via `curl https://crates.io/api/v1/crates/adora-*` — all 404). The fork's local crate names exist only inside the fork's own workspace; nobody outside the fork ever depended on `adora-rs` / `adora-node-api` / `adora-cli` as a crates.io name. Upstream's `dora-*` names win by default, and there is no downstream Rust user to shim for.
> - **No `dora::compat::v0` module needed.** The fork is a strict public-API superset of upstream (verified during Phase 1 merge — every `pub` symbol from upstream `dora-node-api` / `dora-operator-api` is present in the fork with identical signatures). Fork-era POC additions are new surface, not renames of old surface. A 0.x user upgrading to 1.0 by bumping their `dora-node-api` dep to `1.0.0` gets the upstream surface they already depended on, plus the fork extensions as net-new APIs.
> - **PyPI shim IS needed.** `adora-rs 0.2.1` was published to PyPI during the fork era (verified 2026-04-17 at https://pypi.org/project/adora-rs/). That is the one real compat target.
>
> This collapses Phase 4 from 2 days to ~0.5 day.

**Tasks:**

1. **PyPI shim `adora-rs 0.3.0`.** Ship at Phase 5c alongside `dora-rs 1.0.0`. Source lives at `apis/python/compat/adora-rs/` in the tree:
   - `pyproject.toml`: `name = "adora-rs"`, `version = "0.3.0"`, `dependencies = ["dora-rs>=1.0.0,<2.0.0"]`, `Development Status :: 7 - Inactive`.
   - `adora_rs/__init__.py`: emits a `DeprecationWarning` on import, then `from dora import *` so existing imports keep working during migration.
   - `README.md`: one page explaining the rename and pointing at `docs/migration-from-0.x.md`.

   Maintained on PyPI for 6 months after 1.0 GA (until ~2026-11-01), then yanked. No further releases of `adora-rs` on PyPI.

2. ~~**Rust crate-name shims** (`adora-cli` / `adora-node-api` / `adora-operator-api` on crates.io pointing at `dora-*`).~~ **Not needed — never published. See scope correction above.**

3. ~~**`dora::compat::v0` module** with old-signature wrappers.~~ **Not needed — fork is a public-API superset. See scope correction above.**

4. ~~**`dora migrate` / `dora migrate-yaml` subcommands.**~~ **Dropped per #297 resolution, 2026-04-17.** Zero production deployments per `docs/downstream-user-assessment-2026-04-16.md`; `docs/migration-from-0.x.md` documents the manual upgrade steps instead.

**Gate:** `apis/python/compat/adora-rs/` exists in the tree and `python -c "import adora_rs; print(adora_rs.Node)"` against a local install succeeds (import works, `DeprecationWarning` fires, `Node` resolves via the `from dora import *` re-export). Actual PyPI upload of `adora-rs 0.3.0` happens in Phase 5c, after `dora-rs 1.0.0` is live.

### Phase 5a: Release candidate (2 days to tag; stabilization window follows)

**Strategy (2026-04-16 revision):** the original plan ran the dogfood campaign in parallel with Phases 1–4 against the pre-merge fork tree. That's wasted testing — it exercises code that's about to be rewritten (rename + Zenoh SHM + compat layer). Dogfood belongs **on the actual consolidated tree**, so Phase 5 splits into an **RC tag + stabilization window + GA tag**. We eat our own dogfood *as the 1.0-rc*, find the real issues, fix them, then cut GA.

**Phase 5a tasks:**
1. Final CI verification: `cargo test --all`, full E2E, benchmark regression, smoke tests, cargo-audit clean, coverage baseline reported.
2. Merge `v1.0-rewrite` → `main` on `dora-rs/dora` via `--no-ff` merge to preserve the phased history.
3. Tag `v1.0.0-rc.1` on the merge commit. Annotated, signed if release norm is to sign.
4. Push tag. Release workflow fires in **rc mode**:
   - Builds cross-platform CLI binaries.
   - Publishes `dora-* 1.0.0-rc.1` to crates.io with `--allow-features` for pre-release (crates.io allows pre-release versions).
   - Publishes `dora-rs 1.0.0rc1` to PyPI (PyPI pre-release semantics).
   - Creates a **GitHub pre-release** (not a full release), with release notes labelled "release candidate — feedback welcome".
5. Announce the RC only to developer channels (GitHub Discussions, Discord). **No blog post, no HN, no Reddit yet.**

**Phase 5b: RC dogfood + stabilization window (≥ 1 week, open-ended until clean)**

Run the dogfood campaign per [§15 Appendix D](#15-appendix-d-dogfood-campaign-plan) against the 1.0-rc tree. Measurements: uptime, unexpected restarts, shared-memory OOMs, dropped messages, p50/p99/p99.9 latency, memory growth, CPU. Target: 7 clean days (no gate resets allowed — a 4-day run plus a fix plus a new 7-day run is the next attempt, not a 3-day extension).

If the RC surfaces blockers:
- Fix on `main`.
- Tag `v1.0.0-rc.2` (etc.) with each iteration.
- Re-run the 7-day dogfood clock from the new tag.

RC-window exit criteria:
- 7-day clean dogfood run against the most-recent `rc.N` tag.
- No open P0 or P1 bug against the 1.0 series.
- Migration guide validated on at least the fork's own examples and one external repo (if any volunteer from the downstream assessment).

**Gate (Phase 5b):** RC window is closed when dogfood evidence is published, outstanding P0/P1 are zero, and the decision to promote the RC to GA is recorded. No fixed calendar window — ship when it's ready, not when the week ends.

### Phase 5c: GA release (1 day)

Once the RC is stable:

1. Tag `v1.0.0` on the same commit the final `rc.N` pointed at.
2. Release workflow fires in **GA mode**:
   - Re-builds cross-platform CLI binaries with the final version string.
   - Publishes `dora-* 1.0.0` to crates.io (in dependency order).
   - Publishes `dora-rs 1.0.0` to PyPI.
   - Creates the GitHub release (full, not pre-release).
3. After `dora-rs 1.0.0` is live on PyPI:
   - Publish `adora-rs 0.3.0` PyPI shim from `apis/python/compat/adora-rs/` (Phase 4 artifact). Verify `pip install adora-rs` emits the `DeprecationWarning` and pulls in `dora-rs 1.0.0` transitively.
   - **No Rust crate-name shims to publish** — `adora-*` was never on crates.io; see Phase 4 scope correction (2026-04-17).
4. **Announcement:**
   - Blog post on `dora-rs.ai`: "dora 1.0: A Rust-First Rewrite, Built with Agentic Engineering."
   - Post to HN, `/r/rust`, `/r/robotics`, Rust Weekly, Discord.
   - Release note links the dogfood evidence file from the RC window.
5. **Archive the fork repo `dora-rs/adora`** via GitHub Settings (per D-6a). Pin a README pointing at `dora-rs/dora`. Do not delete — external links and the 58 inline `dora-rs/adora#NNN` code-comment refs depend on it. The destination repo `dora-rs/dora` stays active and becomes the canonical 1.0+ home.

**Gate (Phase 5c):** `dora 1.0.0` is publicly released, announcement is live, archive is done.

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
| R-8 | Arrow 54→58 or pyo3 0.23→0.28 breaks downstream Python nodes | High | Medium | Migration guide links upstream changelogs; no `dora migrate` subcommand (dropped per #297) — downstream users follow manual steps in `docs/migration-from-0.x.md` |
| R-9 | Audit critical issues resurface under dora brand | Low (if Phase -1 gate enforced) | Critical | Phase -1 gates on audit closure; independent audit pre-release |
| R-10 | ROS2 bridge users lose functionality | Medium | High | ROS2 bridge superset claim verified in Phase -1; audit dora-unique bridge features |
| R-11 | GitHub issue tracker mass-reports post-release | Medium | Medium | Dedicated post-release triage rotation for 2 weeks; canned responses for common migration questions |
| R-12 | Blog post gets dunked on for "AI rewrote it and broke my stuff" | Medium | Medium | Honest framing; lead with test/verification story not velocity story |
| R-13 | Legacy 0.x branch rots and users on 0.x stop getting security fixes | High (over 6+ months) | Medium | Define explicit EOL date in the migration guide; give 6 months notice |
| R-14 | Merge commit confuses `git blame` or breaks third-party tooling | Low | Low | `git log --first-parent` reproduces the upstream linear timeline; `git blame` after an `ours`-merge attributes each line to the last author who touched it in its native history (i.e. the fork's authors for fork-sourced lines), which is the intended behavior. Document this in the release notes so downstream tools expecting a single linear history aren't surprised. |
| R-15 | Zenoh SHM migration (Phase 3b) regresses latency on small messages | Medium | Medium | Keep 4KB threshold; messages below it continue via existing TCP path; benchmark regression gate blocks release if p99 drops |
| R-16 | Zenoh SHM `unstable` API breaks in a future zenoh release | Low (6-12 months) | Medium | Pin to `~1.8`; monitor zenoh releases; CI nightly rebuild against zenoh's main branch as early-warning |
| R-17 | Zenoh SHM recording (`.drec`) support not finished before release | Medium | High | Phase 3b explicitly includes recording support via a ZShm → Vec<u8> copy at record-node level; gate the release on `examples/validated-pipeline` recording end-to-end test passing |

---

## 7. Decision Points (for maintainer discussion)

These are the decisions that cannot be made unilaterally. Each needs explicit resolution before the relevant phase begins.

### D-0: Consolidation strategy — tree-replacement merge vs staged upstream PRs

**Resolved (2026-04-16):** **D-0a** — tree-replacement merge. The fork tree takes over / overwrites upstream as the dora 1.0 baseline, since the fork is the production superset. Confirmed by heyong4725 in [PR #286](https://github.com/dora-rs/adora/pull/286) following prior contractor briefings with phil-opp and haixuanTao. R-5 (maintainer objection) is retired.

**Context:** the rest of the plan assumes a single tree-replacement merge of `dora-fork` into `dora-upstream`. Alternatives considered and rejected for this decision:

- ~~**D-0b:** Staged upstream PRs over 3–6 months~~ — rejected; superset delta is too large and the consolidation narrative depends on a clean 1.0 cut.
- ~~**D-0c:** Hybrid~~ — rejected; slows down release without meaningfully reducing risk, given contractor buy-in.

**Decision owner (resolved):** heyong4725 with contractor (phil-opp, haixuanTao) concurrence.

---

### D-1: Wire protocol compatibility strategy

**Resolved (2026-04-16): D-1a — hard protocol break.** Evidence: [`docs/phase--1-audit-2026-04-16.md`](phase--1-audit-2026-04-16.md) shows incompatibility at five independent layers (transport tarpc↔WebSocket, top-level dispatch, enum variant ordering, mandatory `Hello` handshake, fork-only `auth.rs`/`ws_protocol.rs` files). Bridge release (D-1b) and version negotiation (D-1c) are not justified for the small user base. Migration guide must document "full cluster restart required for 0.x → 1.0 upgrade"; release note owes an explicit breaking-change callout.

**Context (historical):** fork added `auth.rs` and `ws_protocol.rs` to `libraries/message/`, and most of the existing message files diverged. A 0.5.0 daemon cannot interoperate with a 1.0.0 daemon — connections fail at TCP handshake level.

~~**Options D-1b/D-1c** (rejected):~~
- ~~D-1b: Ship 0.9.0 as a bridge release with both protocols.~~ Cost ~1–2 engineer-weeks; small user base.
- ~~D-1c: Version-negotiation handshake with auto-downgrade.~~ Transport-layer divergence makes this effectively D-1b with extra steps.

**Decision owner (resolved):** heyong4725 based on Phase -1 audit evidence; phil-opp + haixuanTao to confirm on release-note wording.

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

### D-6: Archive or maintain the fork repo post-release

**Context:** post-release, what happens to `dora-rs/adora` (this repo)?

**Options:**
- **D-6a:** Archive with pinned README pointing at `dora-rs/dora`.
- **D-6b:** Delete (not recommended — breaks the 58 `dora-rs/adora#NNN` issue references in code comments and the historical clone URLs in the wild).
- **D-6c:** Keep as a future-experiments sandbox separate from `dora-rs/dora`'s stable release.

**Recommendation:** D-6a. If a future-experiments sandbox is desired, create a new repo (e.g. `dora-rs/dora-next`); don't conflate it with the old fork.

**Decision owner:** heyong4725

### D-7: Zenoh SHM migration scope and timing

**Resolved (2026-04-17): D-7c** — partial migration, data plane only. Keep the 4 control channels on custom shmem until 1.1. Matches what upstream did in `dora-rs/adora#1378`. Confirmed by heyong4725 via #297.

**Follow-up note:** the `shared-memory-server` crate remains in the workspace after 1.0 with reduced surface (data-plane code deleted, control channels retained). This area still needs further hardening work post-1.0 — miri / proptest / fuzz coverage gaps remain on the 4 control channels, and `raw_sync_2` pinned fork dep is still present. Track as 1.1 milestone under `plan-zenoh-shared-memory.md` Phase 3.

**Context (historical):** Added 2026-04-08 after the Q1 POC exposed `shared-memory-server` as the largest concentration of unsafe code in the workspace (~30 blocks) and a QA blind spot (can't be analyzed by miri, proptest, or fuzz because tests call libc's `shm_open`). Upstream dora already adopted Zenoh SHM in `dora-rs/adora#1378`.

~~**Options D-7a / D-7b** (rejected):~~
- ~~D-7a: full migration in Phase 3b (delete `shared-memory-server` entirely).~~ Scope creep risk; further from upstream's chosen approach.
- ~~D-7b: defer all zenoh work to 1.1.~~ Ships the QA gap under the dora brand unchanged.

**Decision owner (resolved):** heyong4725 with contractor (phil-opp, haixuanTao) alignment.

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
| -1 | Governance, prereqs, protocol audit, ownership, criticals | 1 week | — |
| 0 | Preparation (tags, branches, freeze) | 2 days | Phase -1 complete |
| 1 | Mechanical merge | 1 day | Phase 0 complete |
| 2 | Rename pass | 2-3 days | Phase 1 green |
| 3 | Test and example union | 3-5 days | Phase 2 green |
| 3b | Zenoh SHM migration (data plane, D-7c) | 2-3 days | Phase 3 green |
| 4 | Compat layer (PyPI `adora-rs` shim only) | 0.5 day | Phase 3b green |
| 5a | Tag `1.0-rc.1` + pre-release publish | 2 days | Phase 4 green |
| 5b | RC dogfood + stabilization (rc.2, rc.3, ... as needed) | ≥ 1 week, open-ended | Phase 5a tagged |
| 5c | Tag `1.0.0` GA + publish + announcement + archive | 1 day | Phase 5b clean |
| 6 | Post-release support | 6 weeks (ongoing) | Phase 5c complete |

**Critical path:** ~4–5 weeks from Phase -1 kickoff to `1.0-rc.1`, then ≥ 1 week open-ended to GA. The RC window is deliberately uncapped — ship when the dogfood is clean, not when a calendar box expires. Phase 3b adds 2-3 days vs. the original estimate.

**Parallel tracks during Phases 1-4:**
- ~~Dogfood campaign~~ — moved to Phase 5b per 2026-04-16 owner direction.
- Q1-minimum test infrastructure (coverage baseline, cargo-audit, see QA plan)
- Blog post and migration guide drafting
- ~~Downstream user outreach~~ — right-sized to release-note + watch per 2026-04-16 assessment.

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
- [ ] No production user reports an unresolved migration blocker in the first 30 days post-release. ~~(Was: "At least 3 downstream projects have confirmed successful migration to 1.0 via `dora migrate`." Replaced 2026-04-16 after downstream assessment showed zero identifiable production deployments — see `docs/downstream-user-assessment-2026-04-16.md`.)~~

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

> **Historical record only — Phase 2 is complete.** This appendix documented the `adora-*` → `dora-*` rename that landed in PR #186 (675 files, +9,612 / −9,752). After the rename, every row in the tables below has "dora" on both sides and is no longer actionable. The content is preserved so future readers can reconstruct what changed; regenerate the "from" column from the PR #186 diff if needed.

### Crate names (was: `adora-*` → `dora-*`)

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

> **Historical — already executed in PR #186.** The script below is preserved as a reference for future consolidations; running it against the current tree is a no-op.

```bash
#!/bin/bash
# rename-adora-to-dora.sh — run from repo root after Phase 1 merge

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

> **Superseded (2026-04-16)** by [`docs/downstream-user-assessment-2026-04-16.md`](downstream-user-assessment-2026-04-16.md). Code search turned up zero production deployments, so the top-10 outreach protocol is disproportionate. The assessment documents the right-sized alternative (release-note prominence + migration guide + issue-tracker watch). This appendix is kept as historical context for the original planning decision.

### Historical: the original top-10 outreach protocol

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
   - Ask: "Would you be willing to test the manual migration steps on your project before the 1.0 release?"
6. Collect responses. Adjust the migration guide based on feedback. (Historically this section referenced a `dora migrate` subcommand; that was dropped per #297 resolution 2026-04-17 — zero production users surfaced in the assessment.)

**This step is not optional.** The single biggest reputational risk of the consolidation is surprise breakage at the downstream user level. Early outreach turns potential critics into collaborators.

---

## 15. Appendix D: Dogfood campaign plan

> **Rescoped 2026-04-16:** originally specified to run in parallel with Phases 1–4 against the pre-merge fork tree. Changed per owner direction — dogfood runs in **Phase 5b** against the tagged `1.0-rc` on the actual consolidated tree, not against the fork. Pre-merge testing would exercise code that's about to be rewritten (rename + Zenoh SHM + compat layer); post-merge RC testing exercises the actual shipping code. Purpose unchanged: generate evidence for the 1.0 release claim.

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

> **Correction (2026-04-16 review):** the previous revision had several commands where the source/target paths collapsed to the same value (pre-rename they compared `dora-rs/dora` vs `dora-rs/adora`; post-rename they compare the same path twice and produce empty diffs). Commands below are rewritten to explicitly distinguish `UPSTREAM` and `FORK` paths so the evidence is real. Run from any working directory; no `cd` needed.

```bash
# --- Setup: two checkouts side by side ---
UPSTREAM=~/src/dora-rs-upstream          # dora-rs/dora (destination repo)
FORK=~/src/dora-rs/adora                 # this repo (source repo)
# Clone if needed:
# git clone https://github.com/dora-rs/dora  "$UPSTREAM"
# git clone https://github.com/dora-rs/adora "$FORK"

# --- Protocol audit: diff each message file between upstream and fork ---
cd "$FORK"
git remote add upstream https://github.com/dora-rs/dora.git 2>/dev/null || true
git fetch upstream main

for f in cli_to_coordinator.rs common.rs config.rs coordinator_to_cli.rs \
         coordinator_to_daemon.rs daemon_to_coordinator.rs daemon_to_daemon.rs \
         daemon_to_node.rs descriptor.rs id.rs metadata.rs node_to_daemon.rs \
         auth.rs ws_protocol.rs; do
  echo "=== $f ==="
  git diff upstream/main..HEAD -- "libraries/message/src/$f" | wc -l
done

# --- Public API diff using cargo-public-api ---
cargo install cargo-public-api 2>/dev/null || true
( cd "$UPSTREAM" && cargo public-api -p dora-node-api ) > /tmp/upstream-node-api.txt
( cd "$FORK"     && cargo public-api -p dora-node-api ) > /tmp/fork-node-api.txt
diff /tmp/upstream-node-api.txt /tmp/fork-node-api.txt > /tmp/api-diff.txt
wc -l /tmp/api-diff.txt

# --- Downstream user grep (unchanged; searches GitHub globally) ---
gh search code 'dora-node-api' --language rust --limit 50 --json repository \
  | jq '.[] | .repository.nameWithOwner' | sort -u > /tmp/downstream-users.txt

# --- Crate ownership verification (both old and new crate names) ---
for crate in \
    dora-node-api dora-operator-api dora-cli dora-core dora-message \
    dora-node-api-python dora-operator-api-python; do
  echo "=== $crate ==="
  cargo owner --list "$crate" 2>&1 || echo "  not published"
done

# --- PyPI ownership ---
# Both upstream and fork publish to the same two PyPI package names:
#   `dora-rs`      (node API — from apis/python/node/pyproject.toml)
#   `dora-rs-cli`  (CLI      — from binaries/cli/pyproject.toml)
# So there's no old-vs-new collision to detect; the real question is whether
# the PyPI account that currently owns each name is the same entity that
# will publish 1.0. Verify existence + owner for both:
for pkg in dora-rs dora-rs-cli; do
  echo "=== $pkg ==="
  pip index versions "$pkg" 2>/dev/null || echo "  $pkg not on PyPI"
  # Owner check requires a PyPI maintainer session; record the owner list
  # manually from https://pypi.org/project/$pkg/ and compare to this repo
  # org admins below.
  echo "  Owner list: https://pypi.org/project/$pkg/"
done

# --- GitHub org admin list ---
gh api orgs/dora-rs/members --jq '.[] | .login'

# --- Test file counts: upstream vs fork ---
echo "upstream tests: $(find "$UPSTREAM" -name '*.rs' -path '*/tests/*' | wc -l)"
echo "fork tests:     $(find "$FORK"     -name '*.rs' -path '*/tests/*' | wc -l)"

# --- Cargo.toml counts: upstream vs fork ---
echo "upstream Cargo.toml: $(find "$UPSTREAM" -name Cargo.toml | wc -l)"
echo "fork Cargo.toml:     $(find "$FORK"     -name Cargo.toml | wc -l)"
```

Save the outputs as `docs/phase--1-audit-YYYY-MM-DD.md` and attach to this plan.

---

## 17. Appendix F: Protocol audit results

**Audit date:** 2026-04-16 · **Full evidence:** [`docs/phase--1-audit-2026-04-16.md`](phase--1-audit-2026-04-16.md) · **Issue:** closed #288 · **Gate:** §19.7 "Wire protocol audit" = Done

### Scope
- `dora-upstream` examined: 0.5.0, `upstream/main` as of 2026-04-16 (HEAD `52835cff6`)
- `dora-fork` examined: 0.2.1, local `HEAD` as of 2026-04-16

### Headline finding

**The protocols are incompatible at multiple layers.** Rolling upgrade between a 0.5.0 deployment and a 1.0.0 deployment is impossible. Break is at: (a) transport layer — upstream uses tarpc/TCP+JSON, fork uses WebSocket; (b) message enum shape — flat RPC structs became enum-dispatched `ControlRequest`; (c) enum variant ordering — fork inserted new variants mid-enum (bincode tags shift); (d) new mandatory `Hello` handshake fork-only; (e) fork-only files `auth.rs` and `ws_protocol.rs`.

### Message-by-message diff (summary)

| File | Diff lines | Wire-compatible? | Notes |
|---|---:|---|---|
| `cli_to_coordinator.rs` | 383 | **No** | Flat structs → `ControlRequest` enum + `Hello` handshake |
| `common.rs` | 221 | **No** | New types |
| `config.rs` | 567 | **No** | Major additions |
| `coordinator_to_cli.rs` | 224 | **No** | New reply variants |
| `coordinator_to_daemon.rs` | 225 | **No** | State catch-up additions |
| `daemon_to_coordinator.rs` | 229 | **No** | `DaemonEvent`, `NodeStatus`, `FaultToleranceSnapshot`, `NetworkMetrics` added |
| `daemon_to_daemon.rs` | 14 | **No** (minor) | Small but breaking |
| `daemon_to_node.rs` | 148 | **No** | `DaemonCommunication::Shmem` variant inserted; `NodeEvent` gained 4 variants (`InputRecovered`, `NodeRestarted`, `ParamUpdate`, `ParamDeleted`); `DaemonReply::NextEvents` type changed |
| `descriptor.rs` | 1,266 | **No** | YAML descriptor superset |
| `id.rs` | 271 | **No** | New identifier types |
| `integration_testing_format.rs` | 0 | Yes | Identical (test-only) |
| `lib.rs` | 73 | — | Internal re-exports |
| `metadata.rs` | 605 | **No** | `TryFromParameterError` removed; `BufferOffset` + metadata-key constants added. `Parameter` variants in matching order (sole overlap). |
| `node_to_daemon.rs` | 12 | Borderline | One-line addition |
| `auth.rs` | fork-only | **No** | Bearer-token auth; no upstream equivalent |
| `ws_protocol.rs` | fork-only | **No** | WebSocket control plane; upstream speaks tarpc |

### Handshake compatibility

- Does a 0.5.0 daemon recognize a 1.0.0 coordinator? **No.** Transport differs (tarpc vs WebSocket); connection fails at TCP handshake level.
- Does a 1.0.0 CLI recognize a 0.5.0 coordinator? **No.** Same transport mismatch.
- Does version negotiation exist? **Asymmetric.** Fork adds mandatory `ControlRequest::Hello { dora_version }` with semver compatibility check (`libraries/message/src/cli_to_coordinator.rs:193-232`). Upstream has only optional `get_version()` tarpc method. Cross-version connection fails before the handshake can execute.

### Recommendation

**D-1a — hard protocol break.** Migration guide must state "full cluster restart required" for 0.x → 1.0 upgrades. D-1b (bridge release) and D-1c (version negotiation) would cost ~1–2 engineer-weeks for a small user base and are not justified. See the evidence file §6 for action items that must land before Phase 5 (explicit release-note callout, interop failure test, startup log line naming the supported protocol version).

---

## 18. Change log for this document

| Date | Author | Change |
|---|---|---|
| 2026-04-07 | heyong4725 + AI | Initial draft |
| 2026-04-16 | heyong4725 + AI | §19 Fresh Superset Audit + Pre-merge checklist (PR #285) |
| 2026-04-16 | heyong4725 + AI | Review amendments: §1.1 terminology; relabel `dora-upstream` vs `dora-fork`; §3.4 workflow counts corrected; §13/Appendix B marked historical (Phase 2 complete); §19.5 contributor framing corrected; §19.6 unsafe count sourced; §19.7 gate-status honesty pass; D-6/D-7 reordered; R-14 wording fixed; Phase 3b vs D-7 contradiction flagged; D-0 consolidation-strategy decision point added; §19.8 checklist expanded with governance / rename-residue / scope items. See review PR description for full findings. |
| 2026-04-16 | heyong4725 + AI | Review round 2 (follow-up from PR #286 comments): line-3 Status header reconciled with §19.7 (no longer claims "gates cleared"); Phase 5 step 8 archive target corrected (archive fork `dora-rs/adora`, not destination `dora-rs/dora`); §3.1 and Phase 4 compat-layer scope corrected — `apis/rust/compat/` does not exist in either tree, so Phase 4 is "create" not "invert"; §16 Appendix E audit commands rewritten with explicit `UPSTREAM` / `FORK` paths (prior version diffed same file against itself); §19.6 unsafe-review enforcement rewritten (the `**/unsafe*` CODEOWNERS glob does not match actual unsafe files — proposed path-scoped CODEOWNERS + content-based CI check); §19.3 dropped upstream #1610 from the gap list (PR was ported *from* the fork per its own body); 4 gaps → 3 gaps. |
| 2026-04-16 | heyong4725 + AI | Review round 3 (follow-up from second PR #286 review): §19.6 action list and §19.8 checklist updated to use path-scoped CODEOWNERS + content-based CI (they still referenced the broken `**/unsafe*` glob as the required step); Appendix E PyPI ownership check fixed (was running `pip index versions dora-rs` twice — verified both upstream and fork publish the same two names `dora-rs` and `dora-rs-cli`, so the check now loops both names and records owner-list URLs for manual ownership verification); §19.6 "Unsafe code isolation" softened from "isolation rule holding" to "target state documented; current compliance partial" with a verified example (channel.rs send_raw / receive / disconnect / data_len / data have inline `unsafe { }` in safe methods and 0 `# Safety:` docs). |
| 2026-04-16 | heyong4725 + AI | Review round 4 (PR #286 third review): §1 headline top-contributor row — prior cell listed phil-opp 1,786 / haixuanTao 1,828 for the `dora-upstream` column but those are `dora-fork` local git-shortlog numbers. Verified actual upstream contributions via `gh api repos/dora-rs/dora/contributors` = phil-opp 2,023 / haixuanTao 1,895. Replaced with the upstream-authoritative figure and added a footnote naming the metric ("GitHub contributions metric" vs `git shortlog`). §19.5 contributor-count row — "upstream 115 / fork 90" had no traceable provenance; upstream contributors API returns 91. Replaced with both methods named, both commands reproducible, and a note that the prior 115/90 framing is not to be reused. |
| 2026-04-16 | heyong4725 + AI | Governance resolution: **D-0 resolved as D-0a** (fork tree overwrites upstream as 1.0 baseline) per heyong4725 confirmation; phil-opp and haixuanTao are contractors and were briefed. §1 Status line updated; §7 D-0 marked resolved; §19.7 Governance row flipped to Done and linked to closed #293. Remaining Phase -1 evidence gates (wire-protocol audit, audit criticals re-verification, ownership, outreach, dogfood) still open as #288, #289, #290, #291, #292 — tracked under epic #287. |
| 2026-04-16 | heyong4725 + AI | **D-1 resolved as D-1a** (hard protocol break) based on Phase -1 wire-protocol audit. Evidence: new file `docs/phase--1-audit-2026-04-16.md` with per-file diff of `libraries/message/src/*.rs` between `upstream/main` and local HEAD, transport-layer analysis (tarpc vs WebSocket), handshake analysis (mandatory `Hello` fork-only), and migration-guide action items. §17 Appendix F populated; §19.7 "Wire protocol audit" flipped to Done; §7 D-1 marked resolved with D-1b/D-1c struck through. Closed #288. |
| 2026-04-16 | heyong4725 + AI | **Audit-2026-03-21 critical closure.** New file `docs/audit-2026-03-21-closure.md` records per-finding status across 23 Critical items: 17 fixed (with fix commit + current-HEAD verification — DC1 bounds check, CM2 Sync invariant docs, SEC1/SEC2 URL hardening, DC2 listener shutdown, AC1–AC4 unwrap elimination, Q1–Q4, ET1–ET2, etc.), 3 eliminated by Phase 3b (DC3, CM3, related), 3 explicitly waived (CM1 analyzed correct by Release/Acquire pair; SEC1 hash opt-in with warning + strict-mode deferred to 1.1; SEC3 TLS deferred post-1.0 to reverse-proxy pattern). §3.6 annotated; §19.7 "Security audit" row flipped to Done. Closed #289. |
| 2026-04-16 | heyong4725 + AI | **PyPI + crates.io ownership verification.** New file `docs/ownership-verification-2026-04-16.md`. Queried crates.io public API for 30 workspace crates: 19 published, 11 new. Of the 19 published, 17 have `github:dora-rs:core` team ownership (heyong4725 + 3 others are team members — publish works); 3 crates (`dora-arrow-convert`, `dora-ros2-bridge`, `dora-ros2-bridge-msg-gen`) missing the team and need `cargo owner --add` from haixuanTao (A1). 11 crates new to 1.0 need publish/no-publish decision (A4). PyPI both packages present (`dora-rs` + `dora-rs-cli` at 0.5.0); maintainer list requires manual web-UI inspection (A2–A3). §19.7 PyPI/crates.io row flipped to Done; §19.8 pre-merge checklist expanded with A1–A4. Closed #290. |
| 2026-04-16 | heyong4725 + AI | **Downstream user assessment (scope right-sized).** New file `docs/downstream-user-assessment-2026-04-16.md`. Per owner direction ("dora is mostly POCs, not production"), ran a quick GitHub code-search sweep across Rust + Python + pyproject queries. ~30 external repos surfaced; top-5 are 12–69 stars (YOR-robot, Ekumen/lekiwi, FlagOpen/RoboDriver, kornia/bubbaloop, mofa-studio); **zero production deployments identified**. §14 Appendix C top-10 outreach protocol annotated as superseded; §11 success criterion "3 downstream projects confirm migration via `dora migrate`" replaced by "no unresolved production migration blockers in first 30 days post-release". Right-sized plan: release-note prominence + migration guide + 30-day issue-tracker watch + community ping. Trigger-to-expand defined for surprise production users. §19.7 row flipped to Done. Closed #291. |
| 2026-04-16 | heyong4725 + AI | **Release flow restructured: Phase 5 split into 5a (rc) + 5b (RC dogfood window) + 5c (GA).** Per owner direction, dogfood runs on the tagged `1.0-rc` against the actual consolidated tree, not on the pre-merge fork. Rationale: pre-merge testing exercises code that's about to change (rename + Zenoh SHM + compat layer); RC testing exercises the actual shipping tree. Phase 5b window is open-ended — ships on clean dogfood, not on a calendar box. §5 Phase 5 rewritten as 5a/5b/5c; §10 timeline table updated with the three-tag cadence; §15 Appendix D rescoped to Phase 5b; §19.7 dogfood row marked "Rescoped — not a Phase -1 gate"; §19.8 checkbox flipped to done-via-rescope; §1 Status line now reads "Phase -1 complete". #292 rescoped to RC-window tracker rather than closed. |
| 2026-04-17 | heyong4725 + AI | **Scope reconciliation round — three team decisions via #297, all resolved.** (1) **D-7 = D-7c** (partial Zenoh SHM migration, data plane only; control channels stay on custom shmem until 1.1). §5 Phase 3b body rewritten to match (data-plane tasks only, keep `shared-memory-server` crate with reduced surface, keep `raw_sync_2` until 1.1); §7 D-7 marked resolved with D-7a/D-7b struck through; follow-up 1.1 work noted. (2) **No `dora migrate` command in 1.0** — 1.0 is a hard break; zero production users surfaced in the assessment; cost of migration tooling outweighs value. §5 Phase 4 tasks 5+6 struck through; §4 overlap matrix `File formats` row updated to reflect hard break; §6 R-8 mitigation updated to remove `dora migrate` reference; §14 Appendix C outreach questions updated to ask about manual-migration testing; §19.8 checkbox flipped. (3) **58 `dora-rs/adora#NNN` code-comment refs left as-is** — archived repo serves issues indefinitely under D-6a; lowest-churn option, preserves historical design-discussion trail. §19.8 checkbox flipped. §1 Status line updated to reflect all decision points resolved. Closes #297. |
| 2026-04-17 | heyong4725 + AI | **Phase 3b rescoped — architectural changes move to RC window.** Verification on 2026-04-17 showed the D-7c premise was broken: upstream PR #1378 is still open (not merged; mergeable=false; Copilot-authored 3,000-line experiment), and upstream main doesn't use zenoh SHM at all. Upstream's current state is TCP-control + `shared_memory_extended`-data (per commit `01995ad67` from 2026-03-18 which removed the shmem control channel entirely). Per owner direction, multiple `v1.0.0-rc.N` tags during Phase 5b will progressively land: rc.2 = Option D (adopt upstream's `01995ad67` control-channel cleanup, tracked in #313); rc.3 = zenoh SHM data-plane migration (port or improve PR #1378 based on its state at work-start, tracked in #314). GA ships with both. §5 Phase 3b rewritten as a deferred placeholder. §7 D-7 outcome remains D-7c (partial migration in 1.0) but via rc.N landings, not pre-rc.1. Closes #309. |

---

## 19. Fresh Superset Audit (2026-04-16)

**Status**: Confirms the fork is a true superset. Originally listed 4 gaps; upstream #1610 was dropped after verification (it was ported *from* the fork), leaving **3 real gaps** to close before merge (see §19.3).

### 19.1 Crate-level comparison

**Only in upstream (1 crate):**
- `libraries/communication-layer/request-reply/` -- stub `RequestReplyLayer` trait with TCP backend, marked TODO. Adora implements request/reply via Arrow metadata correlation IDs. **Intentionally superseded.**

**Only in adora (26 crates):** Python CLI API, record/replay nodes, ROS2 bridge node, coordinator-store, log-utils, recording, shared-memory-server, plus 17 example crates. All are net-new features.

### 19.2 Source file gaps in shared crates

| Upstream file | Adora equivalent | Status |
|---|---|---|
| `cli/command/destroy.rs` | Folded into `up.rs` (`dora down`) | No gap |
| `cli/command/version.rs` | `dora --version` via clap; no coordinator version query | **Minor gap** |
| `daemon/hot_reload.rs` (293 lines) | Reload logic inline in `lib.rs:send_reload()` | Feature parity, different structure |
| `daemon/state.rs` (172 lines) | Refactored into `running_dataflow.rs` + `event_types.rs` | No gap |
| `coordinator/listener.rs`, `server.rs`, `tcp_utils.rs` | Replaced by `ws_control.rs`, `ws_server.rs`, `ws_daemon.rs` | Intentional upgrade |

### 19.3 Recent upstream features NOT in adora (gaps to close)

| Feature | Upstream PR | Impact | Action |
|---|---|---|---|
| **DoraNodeBuilder / daemon_port** | #1591 | Rust API: `DoraNode::builder().daemon_port(6789).build()`. Adora has `init_from_env` / `init_from_node_id` but no builder with custom daemon port. | Port before merge. ~50 lines. |
| **CUDA IPC via ctypes** | #1618 | Python: `dora.cuda` replaced numba with ctypes for CUDA IPC. Adora still uses the old numba version. | Port before merge. Python-only change. |
| **C/C++ publish workflow** | #1611 | CI: publishes pre-built C/C++ libraries on release. Adora has no equivalent. | Port the workflow file. |
| ~~C API tracing subscriber~~ | ~~#1610~~ | **Not actually a gap.** Upstream #1610 is titled "fix(c-api): improve safety and correctness of C node API" and its own body states "Ported from dora-rs/adora" — i.e. upstream *imported this from the fork*, so the fork is ahead on this item. Remove from the gap list. | Drop. |

### 19.4 Dependency versions

| Dep | Upstream | Adora | Winner |
|---|---|---|---|
| arrow | 54.2.1 | 58 | Adora |
| pyo3 | 0.23 | 0.28 | Adora |
| zenoh | 1.1.1 | ~1.8 | Adora |
| tokio | 1.24.2 | 1.28 | Adora |

All major deps: adora ahead.

### 19.5 Contributor preservation (issue #214)

Contributor counting has two systematically-different metrics. This section reports both and names the source so the numbers stay auditable.

- **`dora-upstream` — GitHub contributors API** (default-branch contributors only, with `anon=true` pagination): **91 authors** as of 2026-04-16. Command: `gh api "repos/dora-rs/dora/contributors?per_page=100&anon=true" --paginate --jq 'length'`.
- **`dora-fork` — local `git shortlog`**: **90 authors** as of 2026-04-16. Command: `git log --format='%aN' | sort -u | wc -l`. This counts raw commit authorships across all refs, not just the default branch.
- The two numbers disagree with each other by methodology alone. Pre-merge, rerun both on the day of the merge and also produce a local `git shortlog` on a fresh clone of `dora-upstream` so we have apples-to-apples fork-vs-upstream numbers. A prior draft of this document cited "upstream 115 / fork 90" without a traceable method — don't use those figures.
- Note: `haixuanTao` has three email aliases (`haixuanTao`, `haixuantao`, `Haixuan Xavier Tao`) that split 981 + 442 + 405 commits in this repo. A `.mailmap` pass before merge is recommended so `git shortlog -sne` collapses the aliases.
- The consolidation merge recipe (Appendix A) preserves both histories as parents. After merge: `git shortlog -sne HEAD` (with `.mailmap` applied) shows the full union.
- GitHub's contributor card may still under-count after the merge (the UI may not follow `--allow-unrelated-histories` well). Mitigation: generate `CONTRIBUTORS.md` from `git log --format='%aN <%aE>' | sort -uf` and commit it, so attribution is durable regardless of what the UI renders.

### 19.6 AI-generated code QA rules (issue #207)

Per phil-opp's request, the following guardrails must be in place before 1.0:

1. **Unsafe code isolation (target state — not yet fully met)**: The rule phil-opp asked for is that every `unsafe` block sits in a small, single-purpose function with a documented `# Safety:` contract. Current `unsafe` footprint (grep for `unsafe fn|impl|trait|{` on 2026-04-16): **177 occurrences across 34 files**, concentrated in `libraries/shared-memory-server/` (26), `libraries/extensions/ros2-bridge/src/_core/` (34), and the C/C++ API surfaces.

    **Gap against the rule, verified on 2026-04-16:** several safe methods contain inline `unsafe { }` blocks rather than delegating to a dedicated unsafe helper — e.g. `libraries/shared-memory-server/src/channel.rs` has inline `unsafe` in the otherwise-safe `send_raw`, `receive`, `disconnect`, `data_len`, `data`/`data_mut` methods, and `grep -B1 'unsafe {' channel.rs | grep -c Safety` returns **0**, i.e. none of those blocks have `# Safety:` doc comments. The 2026-04-16 security audit did not find exploitable bugs in these paths, but it cannot be cited as evidence that the isolation rule is "holding". Treat this item as: **target state documented; current compliance partial; pre-1.0 walkthrough owes either (a) refactor to single-purpose unsafe helpers with `# Safety:` docs, or (b) an explicit waiver per file that stays inline.**
2. **Human review for unsafe changes**: Add a CI check requiring human approval for any PR that modifies `unsafe` code. CODEOWNERS alone cannot express this — `unsafe` lives in ordinary files (`libraries/shared-memory-server/src/channel.rs`, `apis/c/node/src/lib.rs`, `apis/c++/{node,operator}/src/lib.rs`, `libraries/extensions/ros2-bridge/src/_core/*.rs`, `libraries/core/src/metadata.rs`, `binaries/daemon/src/node_communication/mod.rs`, etc.) and no glob like `**/unsafe*` matches them. Two workable options:
    - **Path-scoped CODEOWNERS** covering the directories that contain 95%+ of the `unsafe` footprint: `/libraries/shared-memory-server/ /libraries/extensions/ros2-bridge/src/_core/ /apis/c/ /apis/c++/ /libraries/core/src/metadata.rs`. Still human-maintained (drifts when new unsafe lands elsewhere).
    - **Content-based CI check** that diffs the PR and fails if any `+` line matches `\bunsafe\b` without an accompanying `# Safety:` doc comment and a `ai-unsafe-review` label. More robust; higher one-time implementation cost.

    Recommendation: ship both. CODEOWNERS for the hot directories gives blanket coverage; the CI check catches drift.
3. **Test disabling requires human approval**: Add a CI check that fails if any `#[ignore]` or `#[cfg(not(test))]` is added without an accompanying justification comment. Not yet implemented.
4. **Unwrap budget enforcement**: Already in CI (`.unwrap-budget` file, budget: 185).

**Pre-merge action items for #207:**
- [ ] Add **path-scoped CODEOWNERS** covering the directories that hold ~95% of the `unsafe` footprint (do **not** use the `**/unsafe*` glob — it matches nothing in this repo): `/libraries/shared-memory-server/`, `/libraries/extensions/ros2-bridge/src/_core/`, `/apis/c/`, `/apis/c++/`, `/libraries/core/src/metadata.rs`, `/apis/rust/operator/src/raw.rs`, `/binaries/daemon/src/node_communication/`. Owners: `@phil-opp`, `@haixuanTao` (TBD — confirm availability).
- [ ] Add **content-based CI check**: on every PR, fail if the diff adds any `+` line containing `\bunsafe\b` unless the PR has a `ai-unsafe-review` label and the preceding 5 lines include a `# Safety:` doc comment. Catches drift when new `unsafe` lands outside the path-scoped directories.
- [ ] Add CI check for new `#[ignore]` annotations in test code
- [ ] Document the QA rules in `CONTRIBUTING.md`

### 19.7 Updated Phase -1 gate status

> **Honesty pass (2026-04-16 review):** earlier drafts marked several gates "Done" without linking evidence. Rows below are rewritten to distinguish **Done + evidence attached** from **In progress / no artifact yet**. No gate should be claimed "Done" for the maintainer conversation without a linkable artifact (issue, PR, audit file, CI run).

| Gate | Status | Evidence |
|---|---|---|
| Governance alignment | **Done** (2026-04-16) | phil-opp and haixuanTao are contractors and were briefed. D-0 resolved as **D-0a** (tree takeover). Artifact: heyong4725 confirmation on [PR #286](https://github.com/dora-rs/adora/pull/286) + prior contractor conversations. Closed via #293. |
| Wire protocol audit | **Done** (2026-04-16) | [`docs/phase--1-audit-2026-04-16.md`](phase--1-audit-2026-04-16.md) attached; Appendix F populated from the evidence; D-1 resolved as D-1a. Closed #288. |
| Security audit (re-verify 2026-03-21 criticals) | **Done** (2026-04-16) | [`docs/audit-2026-03-21-closure.md`](audit-2026-03-21-closure.md) walks every Critical finding with fix commit + verification, or explicit waiver. 17 fixed, 3 eliminated by Phase 3b, 3 explicitly waived (CM1 code correct by Release/Acquire pairing; SEC1 opt-in hash with warning, strict mode 1.1; SEC3 TLS deferred post-1.0). No open P0 or P1. Closed #289. |
| Superset verification | **Done** (2026-04-16, revised) | 3 real gaps identified in §19.3 (was 4; #1610 dropped after verification) |
| Upstream alignment (#201) | **Done** (2026-04-15) | 25 PRs audited, 3 shipped — link PR closures |
| CI green | **Done** (2026-04-16) | All platforms, all jobs (link latest green run) |
| PyPI/crates.io ownership | **Done** (2026-04-16) | [`docs/ownership-verification-2026-04-16.md`](ownership-verification-2026-04-16.md) — 17 of 19 published crates have `github:dora-rs:core` team ownership (heyong4725 + phil-opp + haixuanTao + bobdingAI are team members). 3 crates missing the team (`dora-arrow-convert`, `dora-ros2-bridge`, `dora-ros2-bridge-msg-gen`) — haixuanTao runs one `cargo owner --add` per crate before Phase 5. 10 crates not yet published need per-crate publish/no-publish decision. PyPI maintainer list requires manual inspection via web UI (tracked as A2–A3 action items). Closed #290. |
| Downstream user list + outreach | **Done — scope right-sized** (2026-04-16) | [`docs/downstream-user-assessment-2026-04-16.md`](downstream-user-assessment-2026-04-16.md). Code search surfaced ~30 external repos; top-5 are 12–69 stars (YOR-robot, Ekumen/lekiwi, FlagOpen/RoboDriver, kornia/bubbaloop, mofa-studio); **zero production deployments identified**. Top-10 outreach protocol replaced by release-note prominence + migration guide + 30-day issue-tracker watch + community ping. Trigger-to-expand defined for surprise production users. Closed #291. |
| Dogfood campaign | **Rescoped — not a Phase -1 gate** (2026-04-16) | Per owner direction, dogfood runs in **Phase 5b** against the tagged `1.0-rc`, not in parallel with Phases 0–1. Pre-merge testing would exercise code that's about to be rewritten; RC testing exercises the actual shipping tree. Tracked as Phase 5b exit criterion, not Phase -1 blocker. #292 reassigned. |
| CLA / DCO status | Not done | Reconcile whether contributors to either tree signed a CLA (§9 Open Question 8). Blocks contributor attribution strategy. |

### 19.8 Pre-merge checklist (based on this audit + 2026-04-16 review)

Before starting Phase 0, close these gaps. Items marked **(review PR)** were added by the plan review; others are from the original audit.

**Code / superset gaps (original):**
- [ ] Port DoraNodeBuilder / daemon_port from upstream #1591
- [ ] Port CUDA IPC ctypes update from upstream #1618
- [ ] Port C/C++ publish workflow from upstream #1611
- [x] ~~Verify C API tracing subscriber parity with upstream #1610~~ — dropped; #1610 was ported *from* this fork, not a gap (see §19.3)

**Governance / verification:**
- [ ] **Written sign-off from phil-opp and haixuanTao on D-0 consolidation strategy (review PR)** — linked as artifact, not an informal brief
- [x] ~~Verify PyPI/crates.io ownership~~ — done 2026-04-16 (#290 closed). See `docs/ownership-verification-2026-04-16.md`. Four follow-up actions below are the residue.
- [ ] **A1 (ownership):** haixuanTao runs `cargo owner --add github:dora-rs:core` for `dora-arrow-convert`, `dora-ros2-bridge`, `dora-ros2-bridge-msg-gen` (3 crates missing the team)
- [ ] **A2 (ownership):** inspect PyPI collaborator lists for `dora-rs` and `dora-rs-cli` via https://pypi.org/manage/project/<name>/collaboration/
- [ ] **A3 (ownership):** add heyong4725 as Maintainer on PyPI for both packages if not already
- [ ] **A4 (ownership):** decide publish / `publish = false` for each of the 10 not-yet-published workspace crates (see §3 of the ownership doc)
- [ ] **Reconcile CLA / DCO status of agent-authored commits (review PR)** — §9 Open Question 8
- [ ] Fill in Appendix F protocol-audit evidence; downgrade §19.7 Done claims until attached **(review PR)**
- [ ] Re-verify / waive the 3 memory-safety + 1 code-execution findings from `docs/audit-report-2026-03-21.md` **(review PR)**
- [x] ~~Start dogfood campaign (runs in parallel with Phases 0-1)~~ — rescoped 2026-04-16 to Phase 5b (post-RC tag); tracked as Phase 5b exit criterion, not Phase -1 blocker.
- [ ] Produce top-10 downstream user list (§14 Appendix C)

**Policy / repo hygiene:**
- [ ] Add **path-scoped** CODEOWNERS for the directories that contain unsafe code (#207) — the `**/unsafe*` glob does not work
- [ ] Add **content-based** CI check that fails on `+unsafe` lines without a `# Safety:` doc + review label (#207)
- [ ] Add CI check for test disabling (#207)
- [ ] Generate CONTRIBUTORS.md from git history (#214)
- [ ] **Add `.mailmap` to collapse haixuanTao email aliases before `git shortlog` is cited publicly (review PR)**

**Rename residue (originally claimed Phase 2 complete — pass still needed):** **(review PR)**
- [ ] README.md, README.zh-CN.md, Changelog.md — replace `github.com/dora-rs/adora` URLs and `.adorec` filename prose
- [ ] `Cargo.toml` workspace `repository` URL
- [ ] `docs/octos-adora-integration-report.md` — rename file
- [ ] `docs/dora-compatibility.md` — rename to `migration-from-0.x.md` and invert direction (§3.5)
- [x] ~~58 inline `dora-rs/adora#NNN` issue refs across 28 files~~ — **leave as-is** per #297 resolution 2026-04-17. Archived GitHub repos continue to serve issues indefinitely under D-6a, so the links remain resolvable. Not touching is the lowest-churn option and preserves the historical design-discussion trail intact. §5 Phase 5c step 5 is consistent (does not delete the archived repo).
- [ ] `guide/po/*.po` translation catalogs

**Scope reconciliation:** **(review PR)**
- [ ] Phase 3b body vs D-7 recommendation (currently contradicts — see §3b / D-7)
- [x] ~~`dora migrate` subcommand: implement, drop from §11 success criteria, or reassign as a post-1.0 item~~ — **dropped 2026-04-17 (#297)**. Hard break, zero production users, no migration tool ships in 1.0.
- [ ] Merge §19 findings back into §3 and delete the stale duplicates in §3 (see P1-7 in review PR)

## 20. Related documents

- [`plan-agentic-qa-strategy.md`](plan-agentic-qa-strategy.md) — Quality and testing strategy that backs this consolidation. **Must-read companion.**
- [`migration-from-0.x.md`](migration-from-0.x.md) — 0.x → 1.0 migration guide. Renamed from `dora-compatibility.md` and inverted per plan §3.5 (2026-04-16).
- [`audit-report-2026-03-21.md`](audit-report-2026-03-21.md) — Dora technical debt inventory. Criticals must be closed before 1.0.
- [`architecture.md`](architecture.md) — System architecture reference.
- [`testing-guide.md`](testing-guide.md) — Current test infrastructure.
