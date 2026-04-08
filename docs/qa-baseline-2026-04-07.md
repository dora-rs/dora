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
