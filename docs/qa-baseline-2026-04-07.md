# QA Baseline — 2026-04-07

Initial state of quality metrics for dora at commit `333cddb`, captured as
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

Command: `./scripts/qa/mutants.sh --full` (scoped to `dora-core` initially)

**Initial baseline on `dora-core`:**

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
It reflects the reality that much of `dora-core`'s code is either untested
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
- **Target by end of Q1 POC:** 70% mutation score on `dora-core`.
- **Target for other critical crates** (next to run): `dora-daemon`,
  `dora-coordinator`, `dora-message`, `dora-coordinator-store`,
  `shared-memory-server`.

### Reproduce

```bash
# Run once (2-3 hours for dora-core)
cargo mutants --package dora-core --jobs 4 --timeout 120 --output /tmp/dora-core-mutants

# Check scores
wc -l /tmp/dora-core-mutants/mutants.out/{caught,missed,unviable,timeout}.txt

# Check hotspots
awk -F: '{print $1}' /tmp/dora-core-mutants/mutants.out/missed.txt | sort | uniq -c | sort -rn
```

---

## SemVer check (`cargo-semver-checks`)

Command: `./scripts/qa/semver.sh`

**Baseline: v0.2.1 git tag** (dora-* crates are not on crates.io, so we
compare against the last git tag via `--baseline-rev`).

**Result:** all 5 publishable crates PASS (no breaking changes detected).

| Crate | Checks | Result |
|---|---|---|
| `dora-node-api` | 196 | pass |
| `dora-operator-api` | 196 | pass |
| `dora-core` | 196 | pass |
| `dora-message` | 196 | pass |
| `dora-arrow-convert` | 196 | pass |

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

**Update 2026-04-08**: Tier 1 is now fully wired (local + CI), plus T2.1 (property tests on `dora-message`) and T2.3 (miri on `dora-core::metadata`). The remaining work in priority order:

1. **Audit remaining `unsafe` code for bugs similar to `metadata::from_array`** (~1-2 hrs) — the metadata.rs find suggests other unsafe boundaries may have similar issues. Focus on pure-Rust unsafe in `libraries/core`, `libraries/coordinator-store`, `libraries/message`, `apis/rust/node`.
2. **`validate.rs` test improvements** (1-2 days) — biggest mutation score gain available (91 missed mutants at 67% coverage).
3. **Dogfood campaign** (1 week background) — needed for any future 1.0 release claim.
4. **Diff coverage gate** (~30 min) — `diff-cover` integration for the 80%-on-touched rule.
5. **T2.4 Fault injection expansion** (~1 day) — add the 4 chaos scenarios in the QA plan's Section 6.4.
6. **T2.2 cargo-fuzz on `ws_protocol`** (~1-2 hrs) — extends from the proptest work.

**Strategic follow-up (not immediate):**
- Adopt Zenoh shared memory to close the `shared-memory-server` QA blind spot (30 unsafe blocks, miri-uncoverable). Sequenced into the dora 1.0 consolidation as Phase 3b — see `plan-dora-1.0-consolidation.md#phase-3b`.

---

## POC case studies (2026-04-08 update)

The first full POC pass produced six distinct findings. Summary table:

| # | Finding | Caught by | Severity | Commit |
|---|---|---|---|---|
| 1 | `time 0.3.45` DoS (RUSTSEC-2026-0009) | `cargo-audit` | Medium 6.8 | `333cddb` |
| 2 | `lru 0.12.5` unsoundness (RUSTSEC-2026-0002) | `cargo-audit` | Latent UB | `333cddb` |
| 3 | `types_match` tautological tests | `cargo-mutants` | Low | `98c6639` |
| 4 | `.cargo/mutants.toml` regex pinned to line:col | Adversarial LLM review | Low | `3ff3785` |
| 5 | `WsResponse { Some(Null) }` serde asymmetry | Property testing | Low | `28c99b3` |
| 6 | **`metadata::from_array` double off-by-one** | **Reading code while writing miri tests** | **High** | `d12e6b8` |

**4 of 6 findings** would have been missed by every gate that existed in dora prior to this POC session. See [`plan-agentic-qa-strategy.md`](plan-agentic-qa-strategy.md) Section 10a for the full analysis and lessons learned, including:
- Miri/FFI limitation (shared-memory-server is not miri-runnable)
- "Infrastructure as forcing function" meta-finding
- Proptest scope discipline
- Unwrap budget script false-positive mode

### Updated metric snapshots

| Metric | Initial (2026-04-07) | End of POC session (2026-04-08) | Delta |
|---|---|---|---|
| Line coverage (dora-core) | 33.85% | — (unchanged, pending re-run) | — |
| **Mutation score (dora-core)** | **37.2% (149/400)** | **43.4% (173/399)** | **+6.1pp, +24 caught** |
| Mutation score (dora-message) | — | 38.1% (59/155) | new baseline |
| Mutation score (dora-coordinator-store) | — | 33.0% (29/88) | new baseline |
| Mutation score (dora-coordinator) | — | 26.1% (73/280) | new baseline |
| Mutation score (dora-daemon) | — | **5.8% (24/413) — misleading, see note below** | new baseline (caveat) |
| Unwrap count (production code, corrected script) | 622 (reported) | **212 (true)** | 684→212 after script fix; old script counted test/bench code |
| `cargo-audit` real findings | 2 (time DoS + lru unsoundness) | 0 | both fixed |
| Open audit critical issues | per 2026-03-21 report | pending re-check | TBD |
| Tier 1 gates wired | 0 | 6 of 6 | fmt, clippy, test, audit, unwrap, coverage, semver |
| Tier 2 gates with baselines | 0 | 3 of 4 | proptest (ws_protocol), miri (metadata), mutation (dora-core + message + coordinator-store); fault injection planned but not implemented |
| Adversarial review setup | no | yes (local; CI pending API secret) | — |
| Case studies with fixes landed | 0 | 8 (+22 mutation-score tests) | — |
| Diff coverage gate | no | yes (70% threshold on PRs) | — |
| Dogfood campaign | undesigned | designed (`plan-dogfood-campaign.md`) | — |
| Fault injection suite | 3 tests | 3 tests + 8 scenarios designed (`plan-fault-injection.md`) | — |

### Mutation-score hotspots by crate

**dora-core** (173 caught / 226 missed / 46 unviable = 445 total):
- Top remaining missed in `validate.rs` (now 69, was 91): `validate_ros2_qos`, `send_stdout_as`, `max_rotated_files`
- `inference.rs`, `build/git.rs`, `descriptor/visualize.rs` — zero or near-zero coverage

**dora-message** (59 caught / 96 missed / 47 unviable = 202 total):
- `coordinator_to_cli.rs` Display impls, `DataflowList::get_active`
- Worth a focused case study similar to `types_match` — likely 50+% gain possible

**dora-coordinator-store** (29 caught / 59 missed / 7 unviable = 95 total):
- **52 of 59 missed mutants are in `redb_store.rs`** — the persistence layer
- Critical code path with weak test coverage. Potential off-by-one / transaction-boundary bugs likely hiding here (same class as `metadata::from_array` finding)
- Recommended follow-up: focused unsafe-audit-style read of `redb_store.rs` while writing tests for it

**dora-coordinator** (73 caught / 207 missed / 54 unviable = 334 total, 26.1% score):
- **115 of 207 missed mutants in `lib.rs`** — the monolithic main module (audit 2026-03-21 flagged this as a "daemon monolith" architectural concern that applies equally here)
- 31 in `state.rs`, 19 in `handlers.rs`
- Expected score, given the architecture: the coordinator is primarily tested via E2E tests that run the whole coordinator as a process. Those tests don't run under cargo-mutants scoped to the coordinator package (see meta-finding below).

**dora-daemon** (24 caught / 389 missed / 90 unviable = 503 total, 5.8% score — **misleading, see meta-finding below**):
- 184 missed in `lib.rs`, 53 in `spawn/prepared.rs`, 21 in `fault_tolerance.rs`
- 16 missed in `bench_support` helper code (unreachable by tests by design — should be excluded from mutation testing)

### Meta-finding: cargo-mutants package scoping loses cross-package E2E coverage

**The low daemon and coordinator scores are misleading.** Running
`cargo mutants --package dora-daemon` only executes tests that live
**inside** the daemon package (`binaries/daemon/tests/`, plus unit
tests in the daemon's own source). The workspace-level E2E tests
that actually exercise the daemon heavily — `tests/ws-cli-e2e.rs`,
`tests/fault-tolerance-e2e.rs`, `tests/example-smoke.rs` — belong
to the `dora-examples` workspace package, not the daemon package.
`cargo test --package dora-daemon` does not run them, and therefore
`cargo mutants` doesn't either.

In practice the daemon is very well-tested by these workspace-level
E2E tests — smoke tests spin up real daemons, fault-tolerance tests
exercise restart policies, ws-cli-e2e tests stress the full CLI →
coordinator → daemon → node path. But those tests are invisible to
mutation testing as currently invoked.

**Implication for the QA plan:** `plan-agentic-qa-strategy.md`
Section 5.2 should be amended. Running cargo-mutants at package
scope is appropriate for pure-library crates (`dora-core`,
`dora-message`, `dora-coordinator-store` — which are tested
primarily by their own unit tests). For the binary crates
(`dora-daemon`, `dora-coordinator`), two options exist:

1. **Run cargo-mutants at workspace scope** with `--package
   dora-daemon` as a filter for which files to mutate, but letting
   `cargo test` discover and run all workspace tests. This is the
   `cargo mutants --workspace --package dora-daemon` pattern (if
   cargo-mutants supports it — check docs).
2. **Move E2E tests into the daemon package** as integration tests
   (`binaries/daemon/tests/*.rs`). Invasive but aligns mutation
   testing with test coverage.

For the POC, we're accepting the misleading score and documenting
the limitation. Follow-up: investigate cargo-mutants workspace-scoped
invocation and re-run the daemon/coordinator baselines.

---

## Change log

| Date | Commit | Change |
|---|---|---|
| 2026-04-07 | 333cddb | Initial baseline captured |
| 2026-04-08 | d12e6b8 | Added 6 case studies from first POC session + updated next-steps |
