# QA Baseline — 2026-04-07

Initial state of quality metrics for adora at commit `333cddb`, captured as
the starting point for the QA strategy in
[`plan-agentic-qa-strategy.md`](plan-agentic-qa-strategy.md).

All subsequent QA work is measured against these numbers. Gates and
ratchets start from here.

---

## Summary

| Metric | Value | Notes |
|---|---|---|
| Workspace crates | ~45 | excludes Python packages from the workspace build |
| Rust source files | 252 | `libraries/`, `binaries/`, `apis/` |
| Cargo.toml files | 63 | including examples and tests |
| MSRV | 1.88.0 | bumped from 1.85.0 to resolve time 0.3.47 DoS |
| CI toolchain | 1.92 | per `.github/workflows/ci.yml` |
| `unsafe` blocks | 185 | mostly in `libraries/shared-memory-server` |
| `unwrap()` + `expect()` count | 622 | non-test, non-example code |
| Open GitHub issues | track in tracker | — |

---

## Coverage (`cargo llvm-cov`)

Command: `./scripts/qa/coverage.sh`

**Workspace totals (excluding Python packages + examples):**

| Metric | Covered / Total | % |
|---|---|---|
| Regions | 18,347 / 53,763 | **34.13%** |
| Functions | 1,231 / 3,268 | **37.67%** |
| Lines | 11,910 / 35,185 | **33.85%** |
| Branches | — | (not enabled) |

**Interpretation:** 34% line coverage is a low but expected baseline. The
gap is dominated by binaries (CLI, daemon, runtime, coordinator) and their
supporting modules, which are tested primarily via smoke and E2E tests
rather than unit tests — and smoke/E2E runs are not instrumented by
`cargo-llvm-cov` as currently configured.

Lifting the baseline is a goal, but not an immediate one. More important
is **diff coverage** on new code (see gate policy below).

### Well-covered modules (>= 80%)

| Crate / Module | Line Cov |
|---|---|
| `libraries/extensions/telemetry/metrics` | 100.00% |
| `libraries/message/integration_testing_format` | 100.00% |
| `libraries/message/ws_protocol` | 98.86% |
| `libraries/core/descriptor/expand` | 91.35% |
| `libraries/core/types` | 90.61% |
| `libraries/recording` | 90.22% |
| `libraries/extensions/ros2-bridge/msg-gen/parser/ident` | 96.25% |
| `libraries/extensions/ros2-bridge/msg-gen/parser/types` | 90.53% |
| `libraries/extensions/ros2-bridge/msg-gen/parser/message` | 90.11% |
| `libraries/shared-memory-server/lib` | 94.78% |
| `libraries/log-utils` | 95.87% |
| `libraries/message/metadata` | 96.22% |
| `libraries/arrow-convert/into_impls` | 90.74% |
| `libraries/coordinator-store/redb_store` | 85.63% |
| `libraries/coordinator-store/in_memory` | 84.71% |
| `libraries/shared-memory-server/channel` | 82.85% |

### Zero-coverage hotspots (highest risk)

Modules with 0% line coverage that contain non-trivial logic. These are
the most urgent gaps:

| Module | Lines | Notes |
|---|---|---|
| `libraries/extensions/ros2-bridge/arrow/serialize/*` | ~1260 | **Critical** — active ROS2 bridge path for robotics workloads |
| `libraries/extensions/ros2-bridge/arrow/deserialize/*` | ~560 | **Critical** — same |
| `libraries/extensions/ros2-bridge/arrow/lib` | 99 | |
| `libraries/core/src/build/*` | ~670 | Build tooling (git source, logger, build command) |
| `libraries/core/src/lib` | 119 | Top-level module glue |
| `libraries/core/src/descriptor/visualize` | 334 | YAML descriptor visualization |
| `libraries/core/src/topics` | 63 | |
| `libraries/extensions/download/lib` | 114 | **Security-sensitive** — URL downloads without integrity verification per 2026-03-21 audit |
| `libraries/extensions/ros2-bridge/msg-gen/types/*` | ~1100 | Message type generation |
| `libraries/extensions/ros2-bridge/src/_core/*` | ~270 | |
| `binaries/runtime/src/operator/*` | ~500 | Operator loading and execution (dynamic lib) |
| `libraries/shared-memory-server/src/bin/bench` | 156 | Benchmark binary (acceptable) |

**Priority actions** (not part of this baseline commit — follow-up work):

1. **Add unit tests to `libraries/extensions/ros2-bridge/arrow/*`** — highest
   priority. This is core production functionality with zero coverage. Aim
   for 70% line coverage by end of week 2 of the POC.
2. **Add integration tests to `libraries/extensions/download`** — ties into
   the 2026-03-21 audit finding about URL downloads.
3. **Add unit tests to `libraries/core/src/descriptor/visualize`** — low
   risk but easy win.
4. Leave binary crate coverage alone for now — binaries are tested via
   E2E and lifting them is a separate effort.

---

## Security (`cargo audit` + `cargo deny`)

Command: `./scripts/qa/audit.sh`

**Current state: PASS (exit 0)**

Real findings from the first run (all resolved in commit `333cddb`):

| Advisory | Crate | Severity | Resolution |
|---|---|---|---|
| RUSTSEC-2026-0009 | `time 0.3.45` | Medium 6.8 (DoS) | Upgraded to 0.3.47; MSRV bump to 1.88.0 |
| RUSTSEC-2026-0002 | `lru 0.12.5` | Unsoundness | Upgraded ratatui 0.29 → 0.30 |
| License: `CDLA-Permissive-2.0` | `webpki-roots` | Config | Added to `deny.toml` allow list |

Waived unmaintained transitive dependencies (see `deny.toml`):

| Advisory | Crate | Review date | Reason |
|---|---|---|---|
| RUSTSEC-2025-0141 | `bincode 1.x + 2.x` | 2026-09 | Workspace dep; migration to postcard or bincode2 planned post-1.0 |
| RUSTSEC-2025-0057 | `fxhash` | 2026-07 | Transitive, waiting for upstream |
| RUSTSEC-2025-0119 | `number_prefix` | 2026-07 | Transitive, low risk |
| RUSTSEC-2024-0436 | `paste` | 2026-07 | Transitive via safer_ffi, low risk |

---

## Mutation testing (`cargo-mutants`)

Command: `./scripts/qa/mutants.sh --full` (scoped to `adora-core` initially)

**Initial baseline on `adora-core`:**

| Metric | Value |
|---|---|
| Total mutations | 431 |
| Viable (built successfully) | 400 |
| **Caught (test detected)** | **149** |
| Missed (test did not detect) | 251 |
| Unviable (won't compile) | 31 |
| Timeout | 0 |
| **Mutation score** | **37.2%** |

**Interpretation:** A 37% mutation score is a low baseline but an honest one.
It reflects the reality that much of `adora-core`'s code is either untested
(matching the coverage gaps) or tested by assertions that pass regardless of
whether the implementation is correct.

### Missed-mutant hotspots by file

| File | Missed | Line coverage | Signal |
|---|---|---|---|
| `libraries/core/src/descriptor/validate.rs` | 91 | 67% | Biggest absolute gap; many validation paths have happy-path only tests |
| `libraries/core/src/inference.rs` | 36 | — | Needs investigation |
| `libraries/core/src/types.rs` | 32 | **90%** | **Tautological tests — 90% line coverage but 32 uncaught mutations. Classic AI-agent failure mode.** |
| `libraries/core/src/descriptor/visualize.rs` | 25 | 0% | No tests exist |
| `libraries/core/src/build/git.rs` | 21 | 0% | No tests exist |
| `libraries/core/src/descriptor/mod.rs` | 16 | 30% | Needs tests |
| `libraries/core/src/lib.rs` | 9 | 0% | No tests |
| `libraries/core/src/descriptor/expand.rs` | 9 | 91% | Edge-case gaps |
| `libraries/core/src/build/build_command.rs` | 6 | 0% | No tests |

### Top-priority test improvements (ranked by ROI)

1. **`types.rs` tautological tests** — highest ROI. High coverage + high escape
   rate means the tests exist but assert trivially. Rewriting these tests to
   verify behavior (not existence) would likely add 20+ caught mutants.
2. **`validate.rs` error-path coverage** — 91 escaped mutants is the biggest
   absolute gap. Adding tests for each validation failure mode (not just the
   happy path) should catch ~50-70 of these.
3. **`descriptor/mod.rs` and `expand.rs`** — smaller gaps, moderate difficulty.
4. **`visualize.rs`, `build/git.rs`, `build/build_command.rs`, `lib.rs`** —
   zero-coverage modules. Add unit tests first, then mutation tests will
   catch things automatically.

### Mutation testing gate policy

Starting from this baseline:

- **Full-repo runs:** weekly in CI (T2.5). Track the score trend in
  `docs/qa-baseline-*.md` updates.
- **PR-scoped runs (`--in-diff`):** eventually gate PRs (T1.2). For now,
  run locally on touched files. Zero escaped mutants in diff is the target
  once tuning is done.
- **Target by end of Q1 POC:** 70% mutation score on `adora-core`.
- **Target for other critical crates** (next to run): `adora-daemon`,
  `adora-coordinator`, `adora-message`, `adora-coordinator-store`,
  `shared-memory-server`.

### Reproduce

```bash
# Run once (2-3 hours for adora-core)
cargo mutants --package adora-core --jobs 4 --timeout 120 --output /tmp/adora-core-mutants

# Check scores
wc -l /tmp/adora-core-mutants/mutants.out/{caught,missed,unviable,timeout}.txt

# Check hotspots
awk -F: '{print $1}' /tmp/adora-core-mutants/mutants.out/missed.txt | sort | uniq -c | sort -rn
```

---

## SemVer check (`cargo-semver-checks`)

Command: `./scripts/qa/semver.sh`

**Baseline: v0.2.1 git tag** (adora-* crates are not on crates.io, so we
compare against the last git tag via `--baseline-rev`).

**Result:** all 5 publishable crates PASS (no breaking changes detected).

| Crate | Checks | Result |
|---|---|---|
| `adora-node-api` | 196 | pass |
| `adora-operator-api` | 196 | pass |
| `adora-core` | 196 | pass |
| `adora-message` | 196 | pass |
| `adora-arrow-convert` | 196 | pass |

Expected result since HEAD == v0.2.1 at baseline capture. Real signal will
surface on subsequent PRs.

---

## Unwrap budget

Command: `./scripts/qa/unwrap-budget.sh`

| Metric | Value |
|---|---|
| Current count | **622** |
| Budget | 622 (`.unwrap-budget`) |

**Scope:** `.unwrap()` and `.expect(` occurrences in `libraries/`,
`binaries/`, `apis/`, excluding `tests/`, `build.rs`, `examples/`.

**Ratchet policy:** The budget can only decrease. Every PR that reduces
the count should lower `.unwrap-budget` in the same commit. Increases
require explicit justification in the commit message and a budget bump.

**Distribution (from the initial scan — for prioritization):**

The highest-density files for unwrap cleanup (candidates for first sweep):
- `binaries/daemon/` and `binaries/coordinator/` — the primary source
  of panics at runtime given their role
- `libraries/shared-memory-server/` — panics here could corrupt IPC state
- `libraries/core/src/descriptor/validate.rs` — large file with many
  `.unwrap()` calls in validation logic

---

## Tests

**Current CI test infrastructure:**
- Unit tests via `cargo test --all` on Linux, macOS, Windows
- Integration tests in `binaries/<name>/tests/`
- Smoke tests: `tests/example-smoke.rs` (networked + local modes)
- E2E tests: `tests/ws-cli-e2e.rs`, `tests/fault-tolerance-e2e.rs`
- Benchmark regression check on every PR
- Typos check

**Not yet in CI (covered by this QA plan):**
- Coverage (established locally, not yet gated in CI)
- Mutation testing
- Property-based testing
- Fuzzing
- `miri` on unsafe crates
- Adversarial LLM review
- SemVer check

---

## Gate policy (going forward)

Starting 2026-04-07, the following policies apply to PRs touching this
repo:

### Blocking (Tier 1)

| Gate | Rule |
|---|---|
| `fmt` | Must pass |
| `clippy -D warnings` | Must pass |
| `cargo test --all` | Must pass |
| `cargo audit` | Must exit 0 (no unwaived advisories) |
| `cargo deny check` | Must pass (advisories + licenses + sources + bans) |
| Unwrap budget | Count must not exceed `.unwrap-budget` |
| Coverage diff | (to be added) ≥ 80% on touched lines, total must not drop > 1pp |

### Soft / advisory (warn but don't block)

| Gate | Rule |
|---|---|
| `cargo semver-checks` | Warn on breaking change; hard-fail after 1.0 |
| Adversarial LLM review | Posts a PR comment; authors must respond |

### Tracked but not gated yet

| Metric | Rule |
|---|---|
| Mutation score | Establish baseline next, then ratchet |
| Branch coverage | Enable in llvm-cov config, then track |
| Unsafe block count | Add budget file, ratchet |

---

## How to reproduce this baseline

```bash
# Install all tools
make qa-install

# Run the fast gate (fmt + audit + unwrap)
make qa-fast

# Run the full gate (includes tests + coverage, ~5-10 min)
make qa-full

# Individual gates
make qa-audit       # cargo-audit + cargo-deny
make qa-coverage    # cargo-llvm-cov (writes lcov.info)
make qa-unwrap      # unwrap budget check
```

To regenerate just the coverage numbers in this file:

```bash
./scripts/qa/coverage.sh 2>&1 | grep TOTAL
```

---

## Next steps

In priority order per [`plan-agentic-qa-strategy.md`](plan-agentic-qa-strategy.md):

1. **T1.2 Mutation testing** — install `cargo-mutants`, run on
   `adora-core` first, establish initial mutation score baseline, triage
   escaped mutants, commit `mutants.toml` skip list.
2. **T1.6 SemVer check** — run `cargo-semver-checks` on the publishable
   crates to see current breakage state vs last published version.
3. **T1.4 Adversarial LLM review** — wire `scripts/qa/adversarial.sh`
   calling Claude API with the prompt template from
   `plan-agentic-qa-strategy.md` Appendix C.
4. **Coverage diff gate** — install `diff-cover`, wire the 80%-on-touched
   rule, add as a CI-only gate (local runs skip it).
5. **CI integration** — add the QA jobs to `.github/workflows/ci.yml`
   calling the same `make qa-*` targets used locally.

---

## Change log

| Date | Commit | Change |
|---|---|---|
| 2026-04-07 | 333cddb | Initial baseline captured |
