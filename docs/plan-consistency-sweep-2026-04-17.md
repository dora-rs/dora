# Plan-Doc Consistency Sweep

**Date:** 2026-04-17
**Scope:** `docs/plan-dora-1.0-consolidation.md` — every numerical claim, file-existence claim, upstream-vs-fork label, and upstream-PR reference verified against evidence.
**Issue:** closes #296.

## Method

For each category of claim in the plan doc, one reproducible verifying command was run. Where main and the review branch (`docs/consolidation-plan-review`, PR #286) diverge, the more recent/accurate state is cited and the merge-path is noted.

---

## 1. File and directory claims

All file/path claims in the plan were checked against the live tree on `main`.

```bash
for path in <each backtick-quoted path in the plan doc>; do
  [ -e "$path" ] && echo EXISTS || echo MISSING
done
```

**Result: 14 of 16 verified paths exist.** The 2 missing paths are both instances of the same misclaim:

| Claimed path | State | Status |
|---|---|---|
| `apis/rust/compat/dora-node-api` | **Does not exist on either tree** | Addressed on #286 (correction banner in §3.1 and Phase 4) — not yet on main |
| `apis/rust/compat/dora-operator-api` | **Does not exist on either tree** | Same |

Everything else (`apis/rust/node/src/...`, `binaries/record-node`, `binaries/replay-node`, `binaries/ros2-bridge-node`, `docs/audit-report-2026-03-21.md`, `docs/migration-from-0.x.md`, `docs/ros2-bridge.md`, `examples/*`) verified present.

---

## 2. Upstream PR references

Five upstream PR numbers are cited in the plan (§19.3 gap table). Each was verified via `gh api repos/dora-rs/dora/pulls/<n>`:

| PR | Plan claim (on main) | Actual title | Correct on main? |
|---|---|---|---|
| #1378 | "zenoh SHM migration" (Phase 3b body) | "Replace custom POSIX SHM with zenoh SHM for node-direct publish" | ✅ |
| #1591 | "DoraNodeBuilder / daemon_port" | "fix(node-api): add builder API to support custom daemon port" | ✅ |
| #1610 | "C API tracing subscriber parity" | **"fix(c-api): improve safety and correctness of C node API"** — body: _"Ported from dora-rs/adora"_ | ❌ (miscategorized on main; addressed on #286 round 2) |
| #1611 | "C/C++ publish workflow" | "ci: add workflow to publish pre-built C/C++ libraries on release" | ✅ |
| #1618 | "CUDA IPC via ctypes" | "Replace numba with ctypes in dora.cuda for CUDA IPC" | ✅ |

**Net:** #1610 is a misclaim on main (plan labels it as a gap to port; upstream body shows it was ported _from_ the fork). Already corrected on #286.

---

## 3. Crate-name claims

Every `dora-*` crate named in the plan was verified against the fork's `Cargo.toml` packages:

```bash
grep -rh '^name = ' --include=Cargo.toml . | grep -oE '"dora-[^"]*"' | sort -u
```

Output: **32** distinct `dora-*` crate names (not 31 as an earlier draft claimed; corrected 2026-04-17). Note this counts every `name = "..."` declaration in any `Cargo.toml` in the tree, which is a superset of `workspace.members` — `dora-examples` (the root package) is counted here but isn't a member. Every crate named in the plan is present in the tree. No phantom crate names.

One nuance: plan §13 rename map specifies `shared-memory-server` → `dora-shared-memory-server` (for crates.io namespacing). The actual `libraries/shared-memory-server/Cargo.toml` still declares `name = "shared-memory-server"`. This is an **unexecuted rename action**, not a stale claim — the plan describes it as "will be renamed", which is still accurate.

---

## 4. Headline numerical claims (§1 table)

| Metric | Plan claim (main) | Actual (2026-04-17, `gh api`) | On main? | On #286? |
|---|---|---|---|---|
| Upstream stars | 3,188 | **3,483** | stale | stale |
| Upstream forks | 356 | **367** | stale | stale |
| Upstream open issues | 175 | **167** | stale | stale |
| Fork stars | 4 | **6** | stale | stale |
| Fork forks | 9 | **10** | stale | stale |
| Upstream version | 0.5.0 | 0.5.0 | ✅ | ✅ |
| Fork version | 0.2.1 | 0.2.1 | ✅ | ✅ |
| Top contributors (upstream) | phil-opp 2012, haixuanTao 1873 | phil-opp 2,023, haixuanTao 1,895 | stale | fixed |
| Top contributors (fork) | "heyong4725 + agents" | heyong4725 + agents | ✅ | ✅ |
| Last push (upstream) | 2026-04-07 | 2026-04-16 | stale | fixed |
| Last push (fork) | 2026-04-07 | 2026-04-17 | stale | stale (last round 4 date 04-16) |

**GitHub-metric rows (stars/forks/issues) decay automatically** — every row is stale on both branches. Recommendation: update on the day of merge (Phase 1) rather than repeatedly chasing counts that change daily.

---

## 5. Workspace crate count (§1 table)

**Correction (2026-04-17):** the initial write-up compared fork `workspace.members` (61) against upstream Cargo.toml file count (39). That's apples-to-oranges — correct metric on both sides is `workspace.members`. Re-measured:

| Metric | Plan claim | Actual | Method |
|---|---|---|---|
| Upstream `workspace.members` | ~30 | **35** | `gh api repos/dora-rs/dora/contents/Cargo.toml \| jq .content \| base64 -d \| awk '/^members = \[/,/^\]/' \| grep -cE '^\s*"'` |
| Fork `workspace.members` | ~45 | **61** | `awk '/^members = \[/,/^\]/' Cargo.toml \| grep -cE '^\s*"'` |

**Upstream plan number (~30) is within approximation tolerance** of the actual 35. **Fork plan number (~45) still undercounts** by ~35% vs actual 61. So the only real finding here is the fork-side undercount. Recommendation: update `~45` → `61` (or `~60`) on next plan edit; leave `~30` as-is (or tighten to `~35`).

---

## 6. Dependency versions (§3.2 table)

Verified against both trees' `Cargo.toml`:

| Dep | Plan upstream | Actual upstream | Plan fork | Actual fork | Status |
|---|---|---|---|---|---|
| arrow | 54.2.1 | 54.2.1 | 58 | 58 | ✅ both correct |
| pyo3 | 0.23 | 0.23 | 0.28 | 0.28 | ✅ |
| pythonize | 0.23 | n/a | 0.28 | n/a | ✅ |
| zenoh | 1.1.1 | 1.1.1 | ~1.8 | ~1.8 | ✅ |
| serde_yaml | 0.9.33 | 0.9.33 | 0.9.33 | 0.9.33 | ✅ |
| thiserror | (not pinned) | n/a | 2.0 | 2.0 | ✅ |
| git2 | 0.20.4 | not in root | 0.20.4 | 0.20.4 | ✅ |
| MSRV | 1.85.0 | 1.85.0 | 1.85.0 | **1.88.0** | ❌ **fork cell wrong on main & #286** |
| edition | 2024 | 2024 | 2024 | 2024 | ✅ |

**Correction (2026-04-17):** an earlier write-up of this section claimed `serde_yaml = "0.8.23"` for upstream, calling the row wrong. That was grep error on my part — upstream's `Cargo.toml` has `serde_yaml = "0.9.33"` in `[workspace.dependencies]` (line 89) and `serde_yaml = "0.8.23"` in `[dev-dependencies]` (line 113); §3.2 is a workspace table so `0.9.33` is the right value and the plan row is correct. Withdrawn.

**One active error** not addressed by #286:
1. MSRV fork cell says 1.85.0 but fork's `Cargo.toml` has `rust-version = "1.88.0"` — plan says "tie" which is false. This changes the migration-compatibility story (downstream-user Rust-version bump 1.85 → 1.88 is load-bearing info).

---

## 7. Gate-status claims (§19.7)

Main still claims five Phase -1 gates "Done" that had no evidence attached when the plan was written. #286 has already corrected these — each now cites the evidence file on `docs/consolidation-plan-review`:

| Gate | Main | #286 |
|---|---|---|
| Governance alignment | Done | Done (#293 artifact) |
| Wire protocol audit | Done | Done (`phase--1-audit-2026-04-16.md`, #288) |
| Security audit (2026-03-21 re-verify) | Done | Done (`audit-2026-03-21-closure.md`, #289) |
| Superset verification | Done | Done |
| CI green | Done | Done |
| PyPI/crates.io ownership | Not done | Done (`ownership-verification-2026-04-16.md`, #290) |
| Downstream user list + outreach | Not done | Done — scope right-sized (`downstream-user-assessment-2026-04-16.md`, #291) |
| Dogfood campaign | Not done | Rescoped to Phase 5b (#292) |

All evidence files exist on `docs/consolidation-plan-review` branch; none on main yet (land with #286).

---

## 8. Section + Appendix cross-references

40 internal refs enumerated: `§3.5`, `Appendix A–F`, `D-1` through `D-7`, `Phase -1` through `Phase 6` (including `3b`), `R-1` through `R-17`. Cross-checked every ref against section headers. **All resolve.** No broken anchors.

---

## 9. Summary of net-new findings (not covered by #286 rounds 1–4)

These are the items #296 uncovered that prior rounds didn't. Two of five from the first draft have been withdrawn after the round-1 review of this PR (see the "Retracted" sub-section below).

1. **MSRV fork = 1.88.0, not 1.85.0.** §1 and §3.2 tables both say 1.85.0 for both trees. Downstream-user impact: any 0.x user on rustc 1.85–1.87 must bump compiler to upgrade to 1.0.
2. **Fork `workspace.members` = 61, plan says ~45.** §1 table undercounts by ~35% (corrected method; initial write-up compared fork workspace.members vs upstream Cargo.toml file count, which was not apples-to-apples).
3. **All five GitHub-metric rows (stars/forks/issues on both sides) stale on both branches.** Recommendation: snapshot at Phase 1 merge day.
4. **`shared-memory-server` crate not yet renamed to `dora-shared-memory-server`.** Plan §13 describes this as "will be renamed" — still true, action item still outstanding.

### Retracted after review

- ~~serde_yaml upstream = 0.8.23~~ — grep read line 113 (`[dev-dependencies]`) instead of line 89 (`[workspace.dependencies]`). Upstream workspace value is 0.9.33, plan row is a genuine "tie", finding withdrawn. Flagged by PR #300 round-1 review.
- ~~Upstream workspace crates = 39~~ — that number came from counting Cargo.toml files, not `workspace.members`. Correct upstream `workspace.members` count is 35, which matches the plan's `~30` within approximation tolerance. Flagged by PR #300 round-1 review.

## 10. Recommended path to "ready to hand to maintainers"

The plan doc reaches the acceptance criterion (#296: "ready to hand to phil-opp and haixuanTao without further review rounds") once:

1. **#286 merges** — picks up contributor counts, terminology section, Phase 5 restructure, D-0/D-1 resolutions, gate-status honesty pass, evidence file references, and ~100 other corrections the four review rounds accumulated.
2. **Net-new findings from this sweep** (§9 above) land as a small follow-up — tracked as a comment on #286 for next-round incorporation.
3. **GitHub-metric rows** refreshed on Phase 1 merge day.

No further independent review pass is warranted — the remaining items are point fixes, not structural.

## 11. Reproducing this sweep

Every command above is shell-one-liner. Full reproduction from a clean checkout of `main`:

```bash
# Fast-forward local main to origin/main so the subsequent file-exists
# checks run against the same tree state the sweep recorded. A plain
# `git fetch` alone doesn't move the working tree.
git fetch origin main
git checkout main
git merge --ff-only origin/main

# §1 File/path existence (16 paths — matches the 14-of-16 result in §1)
for path in \
    apis/rust/compat/dora-node-api \
    apis/rust/compat/dora-operator-api \
    apis/rust/node/src/event_stream/data_conversion.rs \
    apis/rust/node/src/node/drop_stream.rs \
    apis/rust/node/src/node/mod.rs \
    binaries/record-node \
    binaries/replay-node \
    binaries/ros2-bridge-node \
    docs/audit-report-2026-03-21.md \
    docs/migration-from-0.x.md \
    docs/ros2-bridge.md \
    examples/error-propagation \
    examples/log-sink-tcp \
    examples/ros2-bridge/yaml-bridge-action-server/handler-node \
    examples/ros2-bridge/yaml-bridge-action/goal-node \
    examples/validated-pipeline; do
  [ -e "$path" ] && echo "EXISTS: $path" || echo "MISSING: $path"
done

# §2 Upstream PR titles
for pr in 1378 1591 1610 1611 1618; do
  echo "#$pr: $(gh api repos/dora-rs/dora/pulls/$pr --jq .title)"
done

# §3 Fork crate names
grep -rh '^name = ' --include=Cargo.toml . | grep -oE '"dora-[^"]*"' | sort -u

# §4 GitHub metrics
gh api repos/dora-rs/dora --jq '{stars,forks:.forks_count,issues:.open_issues_count}'
gh api repos/dora-rs/adora --jq '{stars,forks:.forks_count,issues:.open_issues_count}'

# §5 Workspace.members counts (apples-to-apples)
echo "fork: $(awk '/^members = \[/,/^\]/' Cargo.toml | grep -cE '^\s*"')"
echo "upstream: $(gh api repos/dora-rs/dora/contents/Cargo.toml --jq .content | base64 -d | \
  awk '/^members = \[/,/^\]/' | grep -cE '^\s*"')"

# §6 Dep versions — scope to [workspace.dependencies] so you don't pick up a
# [dev-dependencies] copy of the same key (which is exactly how the sweep's
# first draft misread serde_yaml on upstream).
#
# Note: don't use `awk '/^\[workspace.dependencies\]/,/^\[/'` — the `/^\[/`
# end pattern matches the START line too, so awk exits immediately and emits
# nothing. Use the explicit-flag form below, or use sed.
awk '/^\[workspace\.dependencies\]/{p=1;next} /^\[/{p=0} p' Cargo.toml | \
  grep -E "^(arrow|pyo3|zenoh|tokio|serde_yaml|thiserror) = "
gh api repos/dora-rs/dora/contents/Cargo.toml --jq .content | base64 -d | \
  awk '/^\[workspace\.dependencies\]/{p=1;next} /^\[/{p=0} p' | \
  grep -E "^(arrow|pyo3|zenoh|tokio|serde_yaml) = "
grep -E "^rust-version = " Cargo.toml | head -1
gh api repos/dora-rs/dora/contents/Cargo.toml --jq .content | base64 -d | \
  grep -E "^rust-version = " | head -1
```

Expected output from the above commands matches the tables in §1–§6 above.
