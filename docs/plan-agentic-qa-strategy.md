# Agentic QA Strategy

**Status**: Draft — POC in dora, roll out to dora post-consolidation
**Date**: 2026-04-07
**Author**: heyong4725 (with AI assistance)
**Scope**: How to verify code quality and correctness at high confidence when the code is being authored by AI agents at high velocity. Applies first to `dora-rs/dora` as a POC, then to `dora-rs/dora` after the 1.0 consolidation (see [`plan-dora-1.0-consolidation.md`](plan-dora-1.0-consolidation.md)).

---

## 1. Why AI-generated code needs different verification

Traditional code review catches two things well: intent errors ("did you mean to do X or Y?") and style errors ("this doesn't match our conventions"). AI-generated code rarely has either — agents match style by construction and rarely misunderstand intent when the prompt is clear.

What AI-generated code fails at, consistently, is the stuff that humans also fail at but at a different rate:

1. **Tautological tests.** Agents writing tests after implementation (or in the same session) tend to write tests that mirror the implementation structure rather than the specification. The test passes because it asserts what the code does, not what the code should do. Result: high line coverage, near-zero semantic coverage.
2. **Defensive code that's never exercised.** Agents add `if let Some(x) = ...` branches, error variants, and validation checks that cannot be reached by any input the code will ever receive. This inflates line counts and clutters error paths without improving reliability.
3. **Subtle invariant violations.** Agents are good at local correctness and bad at system-level invariants that span multiple files or require holding distributed state in mind. Example: the PR #130 fix where a `ParamReplaySummary { attempted: 0, failed: 1 }` was synthesized to work around a missing connection — syntactically fine, semantically violated the type's own invariant.
4. **Overconfident output.** Agents present all answers with the same confidence whether they're certain or guessing. A human writing uncertain code naturally hedges ("I think this should work"); agents do not.
5. **Architectural drift.** Different agent sessions arrive at different decisions for similar problems. Over weeks, the codebase accumulates three ways to do the same thing, each used by whichever feature was added by whichever session.
6. **No memory across sessions.** Unlike a human team that accumulates shared understanding of "why we don't do X anymore," agents start fresh every session unless explicitly reminded.

**The antidote isn't more code review.** It's verification signals that agents cannot satisfy by producing more code. Specifically:

- **Mutation testing** — directly measures whether tests can detect bugs. A tautological test has zero mutation score. This is the single highest-value signal for AI-generated code.
- **Coverage with branch metrics** — catches the defensive-code-that's-never-exercised pattern.
- **Property-based and fuzz testing** — generates inputs the author never imagined. Agents can't write tests for inputs they didn't think of.
- **Independent adversarial review** — a different model with a different prompt catches blind spots the author's model shares.
- **Dogfooding** — real workloads exercise system-level invariants that no test suite captures.
- **Architectural fitness tests** — encode "we decided X" as executable tests that fail when drift happens.

These are the core of this plan.

---

## 2. Current state (dora, 2026-04-07)

### 2.1 What exists

**CI jobs (`.github/workflows/ci.yml`):**
- `cargo fmt --all -- --check`
- `cargo clippy --all -- -D warnings` (Python packages excluded)
- `cargo test --all` on ubuntu-latest, macos-latest, windows-latest (Python packages excluded)
- E2E tests: `ws-cli-e2e`, `fault-tolerance-e2e`
- Typos via `crate-ci/typos`
- GitHub's GitGuardian security scanner
- Benchmark regression check (latency, memory)

**Test tiers:**
- Unit tests: `#[cfg(test)]` modules in ~252 Rust source files
- Integration tests: `binaries/<name>/tests/`
- Smoke tests: `tests/example-smoke.rs` (networked + local modes)
- E2E tests: `tests/ws-cli-e2e.rs`, `tests/fault-tolerance-e2e.rs`
- Benchmarks: `examples/benchmark/` + benchmark regression CI job

**Release verification:**
- `release.yml` builds cross-platform CLI binaries, Python wheels, publishes to crates.io and PyPI.
- No explicit pre-release QA gate.

### 2.2 What's missing

Critical gaps, ordered by impact on AI-code verification:

| Gap | Impact | Relative cost to fix |
|---|---|---|
| No code coverage measurement | Can't answer "how tested is this?" | Low (1 day) |
| No mutation testing | Can't detect tautological tests | Medium (2-3 days) |
| No property-based testing | Edge cases under-covered | Medium (3-5 days) |
| No fuzzing | Wire protocol, YAML, recording format untested against adversarial input | Medium (3-5 days) |
| No `miri` on unsafe code | 185 `unsafe` blocks, mostly in shared-memory-server, unchecked for UB | Low (1 day nightly) |
| No `cargo-audit` / `cargo-deny` | CVEs ride in via transitive deps | Low (half day) |
| No `cargo-semver-checks` | Accidental breaking changes ship to crates.io | Low (half day) |
| No adversarial LLM review | Single-model blind spots go uncaught | Low (half day) |
| No unwrap budget | 743 `unwrap`/`expect` calls, each a potential panic | Low (1 day initial, ongoing) |
| No architectural fitness tests | Drift across agent sessions goes undetected | Low (1 day per rule) |
| No dogfood campaign | System-level invariants untested in realistic workloads | High (1 week per campaign) |
| No external security audit cadence | Current audit (2026-03-21) is one-shot, no policy for next | Medium (outsourced) |

### 2.3 Audit debt context

Per [`audit-report-2026-03-21.md`](audit-report-2026-03-21.md):
- 3 memory-safety issues (shmem OOB, unsound `Sync`, ARM data race)
- 1 code execution vulnerability (URL downloads without integrity verification)
- 30+ correctness bugs across daemon, coordinator, APIs
- 743 `.unwrap()`/`.expect()` calls in non-test code
- 185 `unsafe` blocks

These are not abstract risks — they are the current cost of shipping without the verification layer this plan proposes.

---

## 3. Philosophy and principles

Five principles drive the design of this plan.

### 3.1 Verification must be measurable, not vibes-based

"We reviewed it" is not a verification signal. "Coverage is 84% with 91% branch, mutation score 76%, 0 cargo-audit advisories, and a 7-day dogfood run with 0 unexpected restarts" is. When reporting quality in release notes, every claim must map to a number.

### 3.2 Signal over ceremony

Every CI job must answer the question "what would fail if this job were removed?" A job that never flags anything is either perfectly tuned (rare) or broken (common). Prefer a small number of high-signal gates to a large number of low-signal ones.

### 3.3 AI agents should treat CI feedback as primary signal

Agents write code, push, see CI results, and iterate. The CI gates are the agent's feedback loop. This means CI must be:
- **Fast** (ideally <10 min for the full PR gate) — long feedback loops degrade agentic quality proportionally
- **Deterministic** — flaky tests teach agents to retry instead of diagnose
- **Actionable** — failure output must tell the agent specifically what to fix, not just "test failed"
- **Explain-able** — agents learn from reading failures; hide too much detail and they stop learning

### 3.4 Prefer verification that catches unknown-unknowns

Unit tests catch known-unknowns (the cases the author thought about). Mutation, property, fuzz, and dogfood catch unknown-unknowns (cases the author didn't think about). AI agents are better at the former than humans and worse at the latter. Weight investment accordingly.

### 3.5 Every verification signal must have an owner

If a CI job starts failing, someone must fix it or escalate. If mutation score drops, someone must investigate. "The agents will handle it" is not an ownership model. For dora 1.0+, the ownership model is: contractor maintainers own all CI signals; each PR that degrades a signal is blocked until the contractor triages.

---

## 4. Three-tier strategy overview

Verification is organized by latency budget, not by kind of tool.

| Tier | Runs | Budget | Purpose |
|---|---|---|---|
| **Tier 1: PR gate** | Every PR | <15 min | Block broken merges; give agents fast feedback |
| **Tier 2: Nightly** | Once per night | <4 hours | Catch slow-to-detect bugs that Tier 1 can't afford |
| **Tier 3: Pre-release** | Per minor version | Days to weeks | Establish the evidence base for a release |

Tier 1 is the agent's primary feedback loop. Tier 2 is the human (or contractor) morning triage. Tier 3 is the release gate.

**Principle:** any check that can run in Tier 1 without violating the time budget should run in Tier 1. Demote to Tier 2 only when cost requires.

---

## 5. Tier 1: PR gate (must add, in priority order)

Target runtime: 15 minutes for the full gate. Current CI is ~12 minutes (Format, Clippy, Tests x3, E2E, Benchmark, Typos). Adding Tier 1 items should push it to ~15 minutes, not 30.

### T1.1 Code coverage with `cargo-llvm-cov` (DAY 1)

**What:** run `cargo llvm-cov` per PR, compute diff coverage against main, fail if diff coverage < 80% OR total coverage drops more than 1%.

**Why:** answers "what fraction of the code is exercised by tests?" Necessary baseline for every other metric.

**Setup:**
```yaml
# .github/workflows/ci.yml — new job
  coverage:
    name: Coverage
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
        with:
          fetch-depth: 0  # needed for diff coverage against main
      - uses: dtolnay/rust-toolchain@master
        with:
          toolchain: 1.92
          components: llvm-tools-preview
      - uses: Swatinem/rust-cache@v2
      - uses: taiki-e/install-action@cargo-llvm-cov
      - name: Generate coverage
        run: |
          cargo llvm-cov --workspace \
            --exclude dora-node-api-python \
            --exclude dora-operator-api-python \
            --exclude dora-ros2-bridge-python \
            --lcov --output-path lcov.info
      - name: Upload to Codecov
        uses: codecov/codecov-action@v4
        with:
          files: lcov.info
          fail_ci_if_error: true
      - name: Diff coverage gate
        run: |
          # Install diff-cover and check diff coverage
          pip install diff-cover
          diff-cover lcov.info --compare-branch=origin/main \
            --fail-under=80
```

**Gate:** diff coverage must be ≥ 80% on touched lines. Total coverage must not drop by more than 1 percentage point vs. main.

**Metrics to publish:** total line coverage, total branch coverage, per-crate coverage in the PR comment.

**POC time:** 1 day in dora. Baseline number will likely be in the 40-60% range given current state.

### T1.2 Mutation testing with `cargo-mutants` on changed files (DAYS 2-4)

**What:** run `cargo-mutants` scoped to files changed in the PR. For each mutation (flipping `==` to `!=`, replacing function bodies with `Default::default()`, etc.), verify at least one test fails. An "escaped mutant" is one where no test caught the change.

**Why:** **This is the single highest-value addition.** It directly measures whether tests are tautological. A test suite with 100% line coverage and 0% mutation score is worse than useless — it gives false confidence.

**Setup:**
```yaml
  mutants:
    name: Mutation testing
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
        with:
          fetch-depth: 0
      - uses: dtolnay/rust-toolchain@master
        with:
          toolchain: 1.92
      - uses: Swatinem/rust-cache@v2
      - uses: taiki-e/install-action@v2
        with:
          tool: cargo-mutants
      - name: Run mutants on diff
        run: |
          cargo mutants --in-diff origin/main \
            --package dora-core --package dora-daemon \
            --package dora-coordinator --package dora-message \
            --timeout 120 \
            --jobs 4 \
            -- --exclude dora-node-api-python \
               --exclude dora-operator-api-python
        continue-on-error: false
```

**Scoping notes:**
- Start with the crates where correctness matters most: `dora-core`, `dora-daemon`, `dora-coordinator`, `dora-message`, `dora-coordinator-store`, `shared-memory-server`.
- Exclude: Python bindings (too slow), examples, tests.
- `--in-diff` scopes mutations to files changed in the PR. Full-repo runs take hours; diff-scoped runs take minutes.
- `--timeout 120` kills tests that take >2 minutes per mutation (catches infinite loops triggered by a mutation).

**Gate:** zero escaped mutants in changed files. If the baseline has escaped mutants, grandfather them in as a `mutants.toml` skip list, but require new code to be clean.

**POC time:** 2-3 days (1 day to wire up, 1-2 days to tune exclusions and initial skip list).

**Expected initial pain:** the first mutation run on dora will find dozens of escaped mutants. This is the point. Triage them into: (a) real test gaps to fix, (b) untestable code to refactor, (c) grandfathered legacy to accept. The skip list should shrink over time.

**Triage guidance (added 2026-04-08 from POC experience)**: initial runs typically find 60-70% escaped mutants on library crates with basic unit test coverage. Sort them into four buckets:

1. **Real test gaps** (most) — function had tests but they weren't thorough. Add focused tests that distinguish each match arm / comparison / boundary. Example: `parse_byte_size` in `dora-core::descriptor::validate` — 14 escaped mutants fixed by writing one test per unit (B/KB/MB/GB).
2. **Tautological tests** (some) — code had high coverage but tests mirror the implementation. Rewrite tests to assert behavior from the spec, not from the implementation. Example: `types_match` in `dora-core::types` — 2 of 3 escaped mutants were in unparseable-URN fallback paths no test exercised.
3. **Equivalent mutants** (rare, 3-10%) — mathematically indistinguishable from the original code. No test can catch them. Document in `.cargo/mutants.toml` with explicit `exclude_re` entry + inline reasoning. See `types_match` `||`→`&&` example.
4. **Cargo-mutants scoping artifacts** — low caught-rate on binary crates like `dora-daemon` (5.8% initially) is an artifact of `cargo mutants --package` not running workspace-level tests. Fix: set `test_workspace = true` in `.cargo/mutants.toml`. Verified 2026-04-08 that `fault_tolerance.rs` goes from 21 missed/0 caught (package-scoped) to 0 missed/21 caught (workspace-scoped).

**`exclude_re` pattern discipline**: when adding an equivalent-mutant waiver, match on the mutation **name** only, never pin to line:column numbers. Line numbers shift when unrelated code above the target is edited, silently breaking the waiver and reintroducing the mutant as noise. Learned in commit `3ff3785`.

Good:
```toml
exclude_re = [
    "libraries/core/src/types\\.rs:.*replace \\|\\| with && in types_match",
]
```

Bad (fragile — line and column pin):
```toml
exclude_re = [
    "libraries/core/src/types\\.rs:177:29.*replace \\|\\| with &&.*types_match",
]
```

**Agent collaboration pattern:** when a PR has an escaped mutant, the agent's next task is "add a test that detects mutation M on file F at line L." This gives the agent a concrete, verifiable subtask. Mutation testing turns into a test-writing prompt generator.

### T1.3 Supply chain gates: `cargo-audit` + `cargo-deny` (HALF DAY)

**What:** `cargo-audit` checks dependencies against the RustSec advisory database. `cargo-deny` enforces license, source, and dependency policies.

**Why:** catches CVEs in transitive dependencies before they merge. Dora's 2026-03-21 audit flagged 2 CVEs — both of these tools would have caught them at PR time.

**Setup:**
```yaml
  supply-chain:
    name: Supply chain
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: taiki-e/install-action@v2
        with:
          tool: cargo-audit,cargo-deny
      - run: cargo audit --deny warnings
      - run: cargo deny check
```

**deny.toml** (new file at repo root):
```toml
[licenses]
version = 2
allow = [
    "Apache-2.0",
    "MIT",
    "BSD-3-Clause",
    "BSD-2-Clause",
    "ISC",
    "Unicode-3.0",
    "Zlib",
    "CC0-1.0",
    "MPL-2.0",
]
confidence-threshold = 0.8

[bans]
multiple-versions = "warn"
wildcards = "deny"

[sources]
unknown-registry = "deny"
unknown-git = "deny"
allow-registry = ["https://github.com/rust-lang/crates.io-index"]

[advisories]
version = 2
yanked = "deny"
```

**Gate:** zero advisories, no disallowed licenses, no yanked crates. Warnings for multiple versions are tolerated but tracked.

**POC time:** half a day.

### T1.4 Adversarial LLM review (HALF DAY + ongoing)

**What:** post a second review on every PR by a different model using an adversarial prompt. Not a replacement for human review — an additional signal.

**Why:** AI agents share blind spots. A different model with a different prompt is the cheapest "second pair of eyes" and catches a surprising fraction of single-model mistakes.

**Setup:**
```yaml
  adversarial-review:
    name: Adversarial review
    runs-on: ubuntu-latest
    if: github.event_name == 'pull_request'
    steps:
      - uses: actions/checkout@v4
        with:
          fetch-depth: 0
      - name: Adversarial review via Claude
        env:
          ANTHROPIC_API_KEY: ${{ secrets.ANTHROPIC_API_KEY }}
        run: |
          # Get the diff
          git diff origin/main...HEAD > /tmp/diff.patch

          # Call Claude API with adversarial prompt
          python scripts/adversarial-review.py /tmp/diff.patch \
            --model claude-opus-4-6 \
            --post-comment \
            --pr ${{ github.event.pull_request.number }}
```

**Prompt template (`scripts/adversarial-review.py`):**
```
You are a skeptical senior reviewer looking at a PR that was likely authored by
an AI agent. Your job is to find issues the agent probably missed. Focus on:

1. **Tautological tests** — tests that mirror implementation instead of testing
   behavior. Flag any test where the test logic looks identical to the code
   logic.
2. **Unreachable defensive code** — error paths, validation, fallbacks that
   cannot be triggered by any realistic input. Flag and ask "when does this
   actually fire?"
3. **Invariant violations** — places where the code constructs a value that
   violates the type's own invariant (e.g., ParamReplaySummary { attempted: 0,
   failed: 1 }).
4. **Silent error swallowing** — `.ok()`, `let _ = ...`, `if let Err(_) = ...`
   that loses information without logging or propagation.
5. **Unwrap in non-test code** — each new `.unwrap()` or `.expect()` is a
   potential panic. Ask: is this actually infallible, or is it lazy?
6. **Concurrent bugs** — lock ordering, shared state without synchronization,
   async tasks that outlive their scope.
7. **Breaking changes not in the migration guide** — public API renames,
   removed variants, changed defaults.
8. **Scope creep** — changes unrelated to the PR title or description.

For each issue, cite the file:line and explain the concern in <200 words.
Be specific. Do not lecture about principles.

If you find no issues, say "No issues found" — do not invent problems.

The diff:
<paste diff here>
```

**Gate:** the review is non-blocking (it's a signal, not a vote). But it posts as a PR comment and the author must respond to each flagged issue before merging.

**POC time:** half a day to wire up, ongoing prompt tuning.

**Cost:** ~$0.10-$0.50 per PR at current API prices. Negligible compared to developer time saved.

### T1.5 Unwrap budget (ongoing ratchet)

**What:** a CI check that counts `.unwrap()` and `.expect(` in non-test, non-build.rs code. Fails if the count exceeds a baseline in `.unwrap-budget`. The baseline can only decrease over time.

**Why:** each unwrap is a potential panic at runtime. Current count is 743. A ratchet lets new code pay down the debt without blocking all work.

**Setup:**
```bash
#!/bin/bash
# scripts/check-unwrap-budget.sh

CURRENT=$(rg '\.unwrap\(\)|\.expect\(' \
  --type rust \
  -g '!**/tests/**' \
  -g '!**/build.rs' \
  -g '!**/examples/**' \
  libraries/ binaries/ apis/ \
  | wc -l)

BUDGET=$(cat .unwrap-budget)

if [ "$CURRENT" -gt "$BUDGET" ]; then
  echo "Unwrap budget exceeded: $CURRENT > $BUDGET"
  echo "Either:"
  echo "  1. Reduce unwraps in your PR, or"
  echo "  2. Update .unwrap-budget with justification in the PR description."
  exit 1
fi

if [ "$CURRENT" -lt "$BUDGET" ]; then
  echo "Unwrap count reduced: $CURRENT < $BUDGET"
  echo "Update .unwrap-budget to $CURRENT and commit."
  # Non-blocking; reminds to ratchet down
fi
```

**Gate:** new unwraps beyond the budget fail the PR. Reductions are celebrated but not auto-applied.

**POC time:** 1 day (half day for the script, half day to establish initial baseline of 743 and land it).

**Ratchet policy:** every PR that reduces the number is encouraged to update `.unwrap-budget` in the same commit. Every contractor-led cleanup week should target a 5-10% reduction.

### T1.6 Fast cargo-semver-checks on public crates (pre-merge)

**What:** run `cargo-semver-checks` on PRs that touch public API crates, comparing against the last published version on crates.io. Flag breaking changes that haven't been labeled as such.

**Why:** AI agents change function signatures without always realizing this is a breaking change. Catching it pre-merge prevents accidental SemVer violations in the release.

**Setup:**
```yaml
  semver:
    name: SemVer check
    runs-on: ubuntu-latest
    if: github.event_name == 'pull_request'
    steps:
      - uses: actions/checkout@v4
        with:
          fetch-depth: 0
      - uses: dtolnay/rust-toolchain@master
      - uses: taiki-e/install-action@v2
        with:
          tool: cargo-semver-checks
      - run: |
          for crate in dora-node-api dora-operator-api dora-core dora-message; do
            echo "=== $crate ==="
            cargo semver-checks check-release -p $crate || FAILED=1
          done
          exit ${FAILED:-0}
```

**Gate:** any breaking change on a publishable crate must be acknowledged in the PR description with a `BREAKING CHANGE:` footer. The check itself is soft (warn) during 0.x development and hard (fail) after 1.0.

**POC time:** half a day. More valuable post-1.0 than pre-1.0 but worth having in place early.

### Tier 1 summary

| Gate | Cost to add | Ongoing cost/PR | Signal quality for AI code |
|---|---|---|---|
| T1.1 Coverage | 1 day | ~2 min | High |
| T1.2 Mutation testing | 2-3 days | ~5-10 min | **Highest** |
| T1.3 Supply chain | 0.5 day | ~30 sec | High |
| T1.4 Adversarial review | 0.5 day | ~1 min + $0.20 | High |
| T1.5 Unwrap budget | 1 day | ~5 sec | Medium |
| T1.6 SemVer check | 0.5 day | ~1 min | Medium |
| **Total** | **~5.5 days** | **~10 min/PR** | — |

**Priority order for POC in dora:** T1.1 → T1.3 → T1.4 → T1.2 → T1.5 → T1.6. Coverage first because it establishes the baseline; supply chain and adversarial review next because they're cheap and high-signal; mutation testing fourth because it's the most valuable but also the most work to tune; unwrap budget and SemVer as the final polish.

---

## 6. Tier 2: Nightly

Target runtime: <4 hours. Runs once per night on main. Failures page the contractor on-call rotation.

### T2.1 Property-based testing with `proptest` (3-5 days to build up, ongoing)

**What:** `proptest` generates randomized inputs to test invariants. For each target, define a property (e.g., "encode then decode is a fixed point") and proptest generates thousands of inputs.

**Why:** catches edge cases no one would think to write. AI agents are particularly weak at generating test inputs they didn't use while writing the code.

**Scope discipline (added 2026-04-08)**: write strategies that test **your** invariants, not properties of the platform your code depends on. Broad strategies like `any::<f64>()` will find failures in the platform (e.g., JSON number precision near the edge, `Some(Value::Null)` vs `None` in serde Option handling) and report them as "bugs" in your code. When a property fails:

1. Reproduce manually with the minimal failing input.
2. Ask: "is this a property of my code, or a property of what my code depends on?"
3. If the latter: constrain the strategy, document why in a comment, and add a regression unit test pinning the equivalence as intentional.
4. If the former: it's a real finding. Fix or document as invariant.

Concrete example from the POC (commit `28c99b3`, `ws_protocol.rs`): `any::<f64>()` produced values like `-5.78e+120` that lose 1 ULP through JSON Number roundtrip. Not an dora bug, a property of JSON. Strategy was constrained to `i32 as f64` (always representable losslessly). Then the *real* finding surfaced: `WsResponse { result: Some(Value::Null) }` does not round-trip through the wire (serde collapses `Some(Null)` to `None`). That was pinned as a documented invariant with a regression unit test.

**High-value targets for dora/dora:**

1. **`dora-message` encode/decode round-trip.** For every message type:
   ```rust
   proptest! {
     #[test]
     fn message_roundtrip(msg: DaemonToCoordinator) {
       let encoded = bincode::serialize(&msg).unwrap();
       let decoded: DaemonToCoordinator = bincode::deserialize(&encoded).unwrap();
       prop_assert_eq!(msg, decoded);
     }
   }
   ```
   Add an `Arbitrary` impl (or derive via `proptest-derive`) for each type. Expect to find at least one serialization bug in dora's current protocol set.

2. **`dora-core` YAML descriptor parsing.** Property: "parsing never panics on any UTF-8 input."
   ```rust
   proptest! {
     #[test]
     fn yaml_parsing_never_panics(input in ".*") {
       let _ = Descriptor::from_yaml_str(&input);  // must not panic
     }
   }
   ```

3. **Coordinator state machine.** Property: "any sequence of {daemon connect, daemon disconnect, node ready, node done, restart} preserves invariant I." I is defined per test: e.g., "every ready node has a registered daemon"; "ack sequence is monotone per daemon"; "restart count is non-decreasing."

4. **Arrow conversion layer.** Property: "Arrow → bytes → Arrow preserves schema and data for all supported types."

5. **Shared-memory framing.** Property: "any sequence of `write_frame(random_bytes)` followed by `read_frame` returns the same bytes."

6. **`.adorec` recording format.** Property: "recording any finite event stream and replaying it produces the same event stream."

**Setup:** add `proptest = "1.5"` to dev-dependencies of target crates. Write the property test files. No CI config changes — proptest runs under `cargo test` automatically. Control budget with `PROPTEST_CASES=1000` in the nightly job.

**Gate:** nightly failure pages the on-call. On PRs, property tests run with a reduced `PROPTEST_CASES=50` to stay in Tier 1 budget.

### T2.2 Fuzzing with `cargo-fuzz` (3-5 days initial + ongoing)

**What:** `cargo-fuzz` runs libFuzzer against targets defined in `fuzz/fuzz_targets/`. Each target is a function that takes arbitrary bytes and does something with them — parse, decode, execute — without panicking.

**Why:** fuzzing finds inputs that cause crashes, hangs, or wrong answers. Essential for any code that handles untrusted or partially-trusted input (which is every protocol boundary).

**High-value fuzz targets:**

1. `fuzz/fuzz_targets/yaml_descriptor.rs` — parse arbitrary bytes as a dataflow YAML.
2. `fuzz/fuzz_targets/ws_protocol.rs` — decode arbitrary bytes as a WebSocket control plane frame.
3. `fuzz/fuzz_targets/topic_data_frame.rs` — decode the `[16-byte UUID][bincode payload]` binary topic frame.
4. `fuzz/fuzz_targets/shared_memory_frame.rs` — parse shared-memory frame headers.
5. `fuzz/fuzz_targets/recording_format.rs` — parse `.adorec` files.
6. `fuzz/fuzz_targets/arrow_ingest.rs` — convert arbitrary bytes to an Arrow RecordBatch.
7. `fuzz/fuzz_targets/daemon_message.rs` — deserialize arbitrary bytes as any `DaemonToCoordinator` variant.

**Setup:**
```bash
cargo install cargo-fuzz
cd libraries/core
cargo fuzz init
cargo fuzz add yaml_descriptor
# edit fuzz/fuzz_targets/yaml_descriptor.rs
```

**Nightly CI:**
```yaml
  fuzz:
    name: Fuzz nightly
    runs-on: ubuntu-latest
    if: github.event.schedule
    steps:
      - uses: actions/checkout@v4
      - uses: dtolnay/rust-toolchain@nightly
      - uses: taiki-e/install-action@v2
        with:
          tool: cargo-fuzz
      - name: Fuzz each target for 10 minutes
        run: |
          for target in yaml_descriptor ws_protocol topic_data_frame \
                        shared_memory_frame recording_format arrow_ingest \
                        daemon_message; do
            cd libraries/core
            timeout 600 cargo fuzz run $target -- -max_total_time=600 || \
              echo "Fuzz target $target found issue"
            cd ../..
          done
      - name: Upload corpus
        uses: actions/upload-artifact@v4
        with:
          name: fuzz-corpus
          path: fuzz/corpus
```

**Gate:** any new crash found overnight opens a GitHub issue automatically and pages on-call.

**Corpus management:** the corpus is a persistent set of interesting inputs. Cache it between runs via GitHub Actions cache. Over weeks the corpus becomes valuable institutional knowledge.

### T2.3 Miri on the unsafe hotspots (1 day)

**What:** `miri` is a Rust interpreter that detects undefined behavior (data races, use-after-free, out-of-bounds, pointer provenance violations). Run on the crates that contain `unsafe` blocks.

**Why:** dora has ~185 `unsafe` blocks at the time of writing, concentrated in `shared-memory-server` (custom POSIX shmem IPC). The 2026-03-21 audit found memory-safety issues in exactly that area.

**Major amendment 2026-04-08**: the initial target list in this plan was wrong. `shared-memory-server` **cannot** be analyzed by miri — every test in that crate calls `ShmemConf::create` which invokes libc's `shm_open`, and miri does not support foreign function calls to POSIX shmem syscalls on any platform. Every test entry point aborts with "unsupported operation" before the unsafe code under test is reached.

**Correct target list (pure-Rust crates with unsafe):**

1. **`libraries/core/src/metadata.rs`** — pointer arithmetic (`ptr.offset_from`). Has focused unit tests added 2026-04-08 (`metadata/tests.rs`). Runs cleanly under miri. This is where POC commit `d12e6b8` found a double off-by-one bug.
2. **`libraries/coordinator-store`** — pure-Rust storage abstraction. Some unsafe in the redb-backed store.
3. **`apis/rust/operator/types/src/lib.rs`** — `slice::from_raw_parts` at the FFI boundary. Runs under miri if you write tests that construct inputs without touching safer_ffi.
4. **`apis/rust/node/src/event_stream/data_conversion.rs`** — `Buffer::from_custom_allocation` with raw pointers; cannot test the `Shmem`-backed path under miri, but the `Vec`-backed path is analyzable.

**The shared-memory-server coverage gap is real and requires a different mitigation.** Two options:

- **Preferred**: adopt Zenoh's native shared memory feature. Upstream dora already did this in `dora-rs/adora#1378`. Deletes ~660 lines and ~11 unsafe blocks from `channel.rs`, eliminates the uncoverable code by removing it. This is sequenced into the dora 1.0 consolidation as Phase 3b — see [`plan-dora-1.0-consolidation.md`](plan-dora-1.0-consolidation.md).
- **Short-term**: refactor `shared-memory-server` to separate the pure-Rust pointer-arithmetic core from the libc wrapper. Test the core under miri + proptest; leave the wrapper thin. ~2-3 days of work.

**Infrastructure-as-forcing-function meta-finding**: when applying miri to a new target, always **write the miri-required unit tests first, then run miri**. Writing the tests forces close-reading of the unsafe code, which is when bugs are most likely to be spotted. Miri itself has never caught a bug in this POC — but the process of preparing it to run has caught two (`metadata::from_array` off-by-one and `dora_send_operator_output` null UB).

**Setup:**
```yaml
  miri:
    name: Miri on unsafe crates
    runs-on: ubuntu-latest
    if: github.event.schedule
    steps:
      - uses: actions/checkout@v4
      - uses: dtolnay/rust-toolchain@master
        with:
          toolchain: nightly
          components: miri
      - run: cargo miri test -p dora-core metadata::tests
      # Other miri-compatible targets to be added as they gain
      # focused unit tests. Do NOT add shared-memory-server — its
      # tests require libc shm_open which miri does not support.
```

**Limitations:** miri is slow and doesn't run FFI code (so tests using `pyo3` or `safer-ffi` or libc syscalls won't work under miri). Scope to pure-Rust code paths only.

**Gate:** miri failure pages on-call and opens an issue.

### T2.4 Expanded fault injection (3 days initial + ongoing)

**What:** extend `tests/fault-tolerance-e2e.rs` with more chaos scenarios. Each test kills a component, corrupts a file, or partitions the network, then verifies the system recovers.

**Why:** distributed-system correctness can only be verified by subjecting the system to failures. AI agents don't naturally write these tests because they require reasoning about system-level invariants.

**New scenarios to add:**

1. **Coordinator crash mid-transaction** — kill coordinator while a daemon's state report is in flight. Restart. Verify consistent state.
2. **Daemon crash mid-replay** — kill daemon while it's replaying persisted params. Restart. Verify no duplicates and no losses.
3. **Coordinator store corruption** — corrupt a random byte in the coordinator's persistent store. Restart. Verify graceful degradation or clean error.
4. **Network partition during state catch-up** — partition daemon from coordinator mid-sync. Heal. Verify eventual consistency.
5. **Clock skew** — inject clock skew between nodes. Verify HLC handles it correctly.
6. **Partial message delivery** — drop messages randomly between daemons. Verify retry and eventual delivery.
7. **Disk full** — fill the coordinator store's disk. Verify the daemon fails cleanly rather than corrupting state.
8. **OOM killer** — kill a node with signal 9 during high memory use. Verify cleanup.

**Framework:** consider using `madsim` (https://crates.io/crates/madsim) for deterministic distributed simulation. It gives you reproducible fault injection with time control.

**Gate:** nightly. Failures open issues automatically.

### T2.5 Full-repo mutation testing (weekly)

**What:** once a week, run `cargo-mutants` on the entire codebase, not just the PR diff. Track mutation score per crate over time.

**Why:** the PR-scoped Tier 1 check only catches regressions in new code. A weekly full run tracks whether the baseline is improving or drifting.

**Setup:**
```yaml
  mutants-full:
    name: Mutation testing (weekly)
    runs-on: ubuntu-latest
    if: github.event.schedule && github.event.schedule == '0 0 * * 0'
    timeout-minutes: 480
    steps:
      - uses: actions/checkout@v4
      - uses: dtolnay/rust-toolchain@master
      - uses: Swatinem/rust-cache@v2
      - uses: taiki-e/install-action@v2
        with:
          tool: cargo-mutants
      - name: Full mutation run
        run: |
          cargo mutants --package dora-core \
                        --package dora-daemon \
                        --package dora-coordinator \
                        --package dora-message \
                        --package dora-coordinator-store \
                        --package shared-memory-server \
                        --jobs 4 \
                        --timeout 120 \
                        --json > mutants.json
      - name: Publish score
        run: |
          python scripts/publish-mutation-score.py mutants.json
```

**Target score:** 80% caught mutants for critical crates. Track weekly. Opening a GitHub issue if the score drops.

### Tier 2 summary

| Gate | Setup time | Runtime | Signal |
|---|---|---|---|
| T2.1 Property tests | 3-5 days | 30 min | High |
| T2.2 Fuzzing | 3-5 days | 70 min | **Highest** for protocol bugs |
| T2.3 Miri | 1 day | 30 min | **Highest** for unsafe code |
| T2.4 Fault injection | 3 days | 20 min | Highest for distributed bugs |
| T2.5 Full mutation | (included in T1.2) | 2-4 hours | Medium (trend signal) |

**POC priority in dora:** T2.3 (miri) first because it's fastest to set up and addresses the known audit gaps; T2.1 (proptest) second because it has high-value targets (protocol); T2.2 (fuzz) third; T2.4 (fault injection) fourth.

---

## 7. Tier 3: Pre-release

Runs per minor version (and per 1.0 release). Gates the release tag.

### T3.1 Independent security audit

**What:** an external security reviewer (different person/firm each time) audits the codebase in the 1-2 weeks before a minor version release. Produces a written report with CVE-style severity ratings.

**Why:** the 2026-03-21 audit was one-shot. A cadence turns it from a defensive reaction into a routine.

**Cadence:** per minor release (0.3.0, 0.4.0, 1.0.0, 1.1.0, ...). Roughly every 2-3 months.

**Scope:** memory safety (unsafe blocks), cryptography (auth tokens, TLS config), input validation (YAML, wire protocols, URL downloads), supply chain (dependency audit), RBAC (if applicable), denial-of-service resistance.

**Budget:** external audits cost real money. Budget accordingly. Alternative for smaller cadence: a contractor not involved in recent commits performs a self-audit.

**Output:** `docs/audit-report-YYYY-MM-DD.md`. Release notes cite the audit by date.

**Gate:** no critical or high issues in the audit may be open at release time. Medium issues are tracked and prioritized.

### T3.2 Architectural fitness tests

**What:** tests that fail when the architecture drifts. These are ordinary Rust tests that parse the workspace and assert properties.

**Examples:**
- `libraries/core` must not depend on `binaries/*`.
- `libraries/message` must not depend on `tokio` (protocol definitions must be runtime-agnostic).
- No crate except `libraries/shared-memory-server` may have more than 5 `unsafe` blocks.
- The workspace must have exactly one `tokio` version across all crates.
- No crate named `*-internal` may have a public export outside the workspace.
- Each crate must have a `README.md`.

**Setup:**
```rust
// tests/architecture.rs
use cargo_metadata::MetadataCommand;

#[test]
fn core_does_not_depend_on_binaries() {
    let metadata = MetadataCommand::new().exec().unwrap();
    let core = metadata.packages.iter()
        .find(|p| p.name == "dora-core").unwrap();
    for dep in &core.dependencies {
        assert!(!dep.name.starts_with("dora-cli"),
                "dora-core must not depend on CLI");
        assert!(!dep.name.starts_with("dora-daemon"),
                "dora-core must not depend on daemon");
    }
}

#[test]
fn protocol_crate_has_no_runtime_deps() {
    // similar logic for dora-message
}

#[test]
fn unsafe_budget_per_crate() {
    // walk libraries/, count unsafe blocks per crate, assert each is under budget
}
```

**Gate:** runs as part of `cargo test --all` in Tier 1. Added here because the rules are typically defined per-release.

### T3.3 Dogfood campaign (1 week before release)

**What:** run a realistic workload on the release candidate for 7 days of continuous operation.

**Why:** the most important pre-release check. System-level invariants (memory leaks, shared-memory exhaustion, lock contention, garbage-collection pauses in Python nodes, file-handle leaks) only show up under sustained load.

**Setup:** see [`plan-dora-1.0-consolidation.md#appendix-d-dogfood-campaign-plan`](plan-dora-1.0-consolidation.md) for the campaign template.

**Gate:** zero unexpected restarts, flat memory curve, p99.9 latency within expected envelope.

### T3.4 Benchmark regression vs. baseline

**What:** already exists in CI via the benchmark regression job. Tier 3 extends it: run the full benchmark suite on the release candidate and compare against the last release's tagged baseline. Any regression >5% on p99 blocks the release.

### T3.5 Coverage, mutation score, audit clean report

**What:** publish a single release-readiness report with:
- Line coverage and branch coverage (with delta vs last release)
- Mutation score per crate (with delta)
- cargo-audit output (must be clean)
- cargo-deny output (must be clean)
- cargo-semver-checks output
- Unwrap budget (current vs. baseline)
- Unsafe block count (current vs. baseline)
- All Tier 2 nightly jobs green for the last 14 days
- All known critical/high issues from the previous external audit resolved

This report is part of the release notes. Users see it. It is the claim-with-evidence that justifies the version number.

### Tier 3 summary

| Gate | Cadence | Cost | Signal |
|---|---|---|---|
| T3.1 External audit | Per minor | Money + 1-2 weeks | Critical |
| T3.2 Architectural fitness | Per release | 1 day per rule | Medium |
| T3.3 Dogfood | Per release | 1 person-week | **Highest** |
| T3.4 Benchmark regression | Per release | Automated | High |
| T3.5 Release-readiness report | Per release | 1 day to assemble | High |

---

## 8. POC plan in dora

The goal of the POC is to validate these tools on the dora codebase, tune them, and produce a known-good CI configuration that can be ported to dora 1.0 post-consolidation.

**POC executed 2026-04-07 to 2026-04-08**: compressed the 5-week phased plan below into a single intensive session. Actual results:

- Tier 1 gates: all 6 wired (fmt, clippy, test, audit, unwrap, coverage+semver soft). CI integration landed on the `main` branch of dora.
- Tier 2 gates: 3 of 4 wired (proptest on `ws_protocol`, miri on `metadata.rs`, mutation baselines on 4 critical crates). Fault injection designed (`plan-fault-injection.md`) but not yet implemented.
- Tier 3 gates: dogfood campaign designed (`plan-dogfood-campaign.md`); first execution tied to the dora 1.0 release cycle.
- Case studies landed: 8 (+22 tautological-test fixes), 3 of which are genuine production bugs (`metadata::from_array` double off-by-one, `dora_send_operator_output` null UB, `NodeId::TryFrom` misleading doc), all caught by focused unsafe-code audit during miri or mutation prep — not by the tools themselves.

The full case-study table, metrics before/after, and meta-findings live in Section 10a of this doc.

**The 5-week phased plan below is retained as a reference for repeating the rollout on dora post-consolidation.**

### Phase P1: Establish baselines (week 1)

Tasks, in order:

1. **Day 1 — Coverage baseline.** Install `cargo-llvm-cov`, run once, capture the number. Don't gate yet; just know where we are. Publish to `docs/qa-baseline-2026-04-07.md`.
2. **Day 1 — cargo-audit + cargo-deny.** Add `deny.toml`. Run both tools. Fix any advisories that surface. Wire as a blocking CI gate.
3. **Day 2 — Unwrap budget script.** Run, capture 743 as baseline. Commit `.unwrap-budget`. Add CI gate.
4. **Day 2 — Adversarial review wiring.** Write the Python script, add the workflow. Test on a sample PR. Tune the prompt based on output.
5. **Day 3-4 — Coverage diff gate in CI.** Add the workflow from T1.1. Test on a throwaway PR. Tune the thresholds.
6. **Day 5 — Initial mutation testing run.** Scope to `dora-core`. Run `cargo mutants --list` to count mutations. Run a small subset to validate the setup. Capture baseline mutation score.

**Deliverable:** `docs/qa-baseline-2026-04-07.md` with all numbers; PR that adds the above to CI.

### Phase P2: Mutation testing and proptest (week 2)

1. **Day 1-2 — Mutation testing on critical crates.** Extend to `dora-daemon`, `dora-coordinator`, `dora-message`. Run to completion. Collect escaped mutants.
2. **Day 3 — Triage escaped mutants.** For each: real test gap / untestable / grandfather. Produce a `mutants.toml` skip list.
3. **Day 4-5 — Property tests for `dora-message`.** Implement round-trip tests for all message types. Find and fix any surfaced bugs.

**Deliverable:** `mutants.toml` baseline; `libraries/message/tests/proptest_roundtrip.rs`; PR that adds mutation testing as a Tier 1 gate.

### Phase P3: Fuzzing and miri (week 3)

1. **Day 1 — cargo-fuzz setup.** Create the `fuzz/` directory, add first target (`yaml_descriptor`).
2. **Day 2-3 — Add remaining fuzz targets.** `ws_protocol`, `topic_data_frame`, `shared_memory_frame`, `recording_format`.
3. **Day 4 — Run each target for 1 hour locally.** Collect crashes. Fix or grandfather.
4. **Day 5 — Miri setup and run on `shared-memory-server`.** Fix any UB findings (these are likely — the 2026-03-21 audit flagged memory-safety issues in this area).

**Deliverable:** `fuzz/` directory wired up; `docs/fuzz-findings-2026-04-15.md`; any fixes landed.

### Phase P4: Fault injection and adversarial review tuning (week 4)

1. **Day 1-3 — New fault injection scenarios.** Add 4 of the scenarios from T2.4 (coordinator crash mid-transaction, daemon crash mid-replay, coordinator store corruption, network partition).
2. **Day 4 — Adversarial review prompt tuning.** Review 10 PR reviews posted so far; identify false positives and false negatives; update the prompt; A/B test.
3. **Day 5 — Full nightly workflow wiring.** All Tier 2 jobs wired up and running. Set up on-call rotation for nightly failures.

**Deliverable:** expanded fault-tolerance test suite; tuned adversarial review prompt; Tier 2 nightly workflow live.

### Phase P5: Dogfood campaign setup (week 5)

1. **Day 1-2 — Design the dogfood workload.** Pick a realistic scenario (e.g., camera → vision model → structured logs across 2 daemons).
2. **Day 3 — Set up monitoring.** Grafana dashboard, Prometheus metrics, structured log collection.
3. **Day 4 — Dry run.** 2-hour run to validate the setup.
4. **Day 5 — Launch the 7-day campaign.** Document the start state.

**Deliverable:** dogfood workload repo; monitoring dashboard; start of the 7-day run.

### POC success criteria

At the end of 5 weeks, dora should have:

- [ ] Tier 1 fully in CI (coverage, mutation, supply chain, adversarial review, unwrap budget, semver)
- [ ] Tier 2 nightly workflow running (proptest, fuzz, miri, fault injection, full mutation weekly)
- [ ] A clean `docs/qa-baseline-2026-04-07.md` with before/after numbers
- [ ] A completed dogfood week with published evidence
- [ ] A `docs/qa-runbook.md` explaining how each gate works and how to diagnose failures
- [ ] A `docs/agentic-qa-lessons.md` documenting what worked, what didn't, and prompt/config tuning that made a difference

**Lessons from the POC feed directly into the dora rollout.** Nothing gets ported to dora that hasn't been validated on dora.

---

## 9. Rollout to dora (post-consolidation)

After dora 1.0 is released (see [`plan-dora-1.0-consolidation.md`](plan-dora-1.0-consolidation.md)), the QA infrastructure validated in dora ports to dora. The port is mostly mechanical — the YAML workflows and scripts move verbatim, with dora → dora find/replace.

### Rollout phases

**Week 1 post-1.0 release — Tier 1 migration:**
- Copy `.github/workflows/ci.yml` additions from dora to dora.
- Copy `deny.toml`, `.unwrap-budget`, `scripts/adversarial-review.py`, `mutants.toml` baselines.
- Re-establish baselines on the dora tree (different commits = different numbers).
- First week of dora PRs run through the full Tier 1 gate.

**Week 2-3 — Tier 2 migration:**
- Copy nightly workflows.
- Re-establish mutation score baseline.
- Re-run miri on the dora tree (may find different UB).
- Re-run fuzz corpus generation.

**Week 4 — Architectural fitness and unwrap ratchet:**
- Port the architectural fitness tests.
- Set new ratchet targets for unwrap budget based on dora 1.0 tree.

**Ongoing:** dora's cadence from this point forward is the same as dora's POC cadence. Agent-authored PRs gate on Tier 1. Nightly Tier 2. Pre-release Tier 3. External audit per minor version.

### What changes at the dora scale

Three things are different about dora vs dora POC:

1. **Higher PR volume.** More contributors, more agents. CI budget becomes more precious. Consider caching mutations by file hash to skip re-running on unchanged files.
2. **Higher scrutiny.** 3188 stars of users will read CI failures in public PRs. Failure messages must be user-friendly, not debug-grade.
3. **More downstream dependents.** SemVer check becomes load-bearing. Every accidental breaking change is a real-world user pain point.

These differences argue for **more investment in making CI output readable and actionable**, not more gates.

---

## 10. Long-term: agentic-engineering-maintained repo

This plan is designed so that dora, post-consolidation, can be maintained primarily by AI agents with contractor oversight rather than full-time human authors. The conditions that make this responsible are:

### 10.1 Non-negotiable prerequisites

1. **Tier 1 gates are load-bearing.** Agents rely on CI as the primary feedback loop. Gates must be fast, deterministic, and actionable.
2. **Contractor owns every signal.** No gate goes un-owned. Failures do not sit in the tracker.
3. **Dogfood campaign per release.** The most agentic-friendly test is still the one that runs the actual product for a week.
4. **External audit per minor version.** Agents can write code faster than humans can review it. External audits are the reality check.
5. **A "why" memory that agents read at the start of sessions.** See section 10.3.

### 10.2 Agent session ergonomics

Each contributor agent session needs, at minimum:
- A `CLAUDE.md` at the repo root with build commands, test commands, lint commands, and failure-mode guidance.
- An `AGENTS.md` (or `docs/for-agents.md`) with guidance specific to agentic engineering patterns: how to run Tier 1 locally, how to interpret mutation testing output, how to respond to adversarial review comments, how to request a dogfood run.
- A `.cursorrules` or equivalent for IDE agents, if used.
- A read-on-start context file pinned to the top of every session prompt. Think of this as the project's "long-term memory."
- Repeated clear decisions documented as ADRs (Architecture Decision Records) in `docs/adr/`.

Dora already has most of this. Dora post-consolidation inherits it.

### 10.3 Architectural memory

Agents don't share memory across sessions. Two agents working on similar problems will reach different solutions unless the project's accumulated decisions are explicit.

**Solution: `docs/adr/` — Architecture Decision Records.** One file per decision. Format:

```markdown
# ADR-NNN: <title>

**Date**: YYYY-MM-DD
**Status**: Proposed | Accepted | Superseded by ADR-MMM
**Deciders**: <names>

## Context
<why this decision was needed>

## Decision
<what was decided>

## Consequences
<what this implies going forward>

## Alternatives considered
<what else was on the table>
```

Examples of ADRs worth writing for dora:
- ADR-001: Use Arrow for all message payloads, not Protobuf.
- ADR-002: Shared memory for >4KB messages, TCP otherwise.
- ADR-003: Zenoh for distributed pub-sub, not raw TCP.
- ADR-004: Auth tokens via bearer header, not query string.
- ADR-005: Coordinator state persists via coordinator-store crate, not file I/O in coordinator directly.
- ADR-006: Mutation testing is a blocking gate, not a nightly-only check.

Every agent session should `Read docs/adr/*.md` at start if it's touching anything architectural. This is the replacement for "the senior engineer in the standup."

### 10.4 Escalation patterns

Agents cannot make the following decisions unilaterally, and CI should flag any PR that attempts them:

- Adding a new dependency to a critical crate (`dora-core`, `dora-message`, `shared-memory-server`).
- Changing a wire protocol.
- Removing a public API.
- Modifying `unsafe` blocks.
- Changing a CI gate's threshold.
- Touching `deny.toml`, `mutants.toml`, or `.unwrap-budget`.

Each requires a contractor sign-off. The PR template should force the author (human or agent) to flag these changes explicitly. CI uses CODEOWNERS to require a contractor review on any touch to these files.

### 10.5 Measuring agentic quality over time

Track these as rolling 30-day averages in a dashboard:
- Lines of code merged per week (velocity)
- Mutation score (trend)
- Coverage (trend)
- Time to first external bug report per release (reliability leading indicator)
- Unwrap budget delta per week (tech debt trend)
- Adversarial review false positive rate (prompt quality)
- Nightly Tier 2 failure rate (signal stability)
- Mean time to green on failing CI (agent learning signal)

If the mutation score or coverage starts dropping while velocity rises, the verification layer is behind and needs investment. If the adversarial review false-positive rate climbs, the prompt needs retuning.

**The core metric is: ratio of external bug reports to lines merged.** If this stays flat or drops as agentic velocity rises, the agentic engineering experiment is working.

---

## 10a. POC findings and lessons learned (2026-04-08)

The first POC pass wired up Tier 1 and part of Tier 2 in a single working session. This section records what happened, what was found, and what it taught us about the plan itself. Corrections to the rest of this document should flow from this section.

### 10a.1 Case studies: findings by gate

Six findings across the QA stack, in commit order:

| # | Finding | Gate that caught it | Severity |
|---|---|---|---|
| 1 | `time 0.3.45` DoS (RUSTSEC-2026-0009) | `cargo-audit` | Medium 6.8 — fixed by upgrade |
| 2 | `lru 0.12.5` unsoundness (RUSTSEC-2026-0002) | `cargo-audit` | Latent UB — fixed by ratatui 0.30 upgrade |
| 3 | `types_match` tautological tests in `dora-core` | `cargo-mutants` | Low — 2 of 3 missed mutants fixed, 1 equivalent mutant documented |
| 4 | `.cargo/mutants.toml` regex pinned to specific line | Adversarial LLM review (claude) | Low — silently breaks if unrelated edits shift code |
| 5 | `WsResponse { result: Some(Value::Null) }` serde asymmetry | Property testing (proptest, first 12 cases) | Low — documented as intentional invariant with regression test |
| 6 | **`metadata::from_array` double off-by-one in region bounds** | **Reading the code while writing miri tests** | **High — rejects valid Arrow buffers at region start / empty buffers at region end** |

**Commits:**
- `333cddb` — security fixes (1, 2)
- `98c6639` — types_match case study (3)
- `3ff3785` — mutants.toml regex fix (4)
- `28c99b3` — ws_protocol property tests + Some(Null) invariant (5)
- `d12e6b8` — metadata::from_array off-by-one fix + miri-runnable tests (6)

### 10a.2 Hierarchy of what each gate actually catches

Based on the six findings above, here is the distinct signal each gate produced that no other gate would have:

| Gate | Uniquely caught in the POC | Caught by other gates too |
|---|---|---|
| Coverage | Nothing uniquely | Established baseline for prioritization |
| `cargo-audit` + `cargo-deny` | Transitive CVEs (2/6 findings) | — |
| Mutation testing | Tautological tests in tested code | — |
| Property testing | `Some(Value::Null)` asymmetry (1/6, would not be caught by any other gate) | Would have caught the f64 edge case too, but that was a proptest-scoping issue not a real bug |
| Adversarial LLM review | Config file fragility (1/6, would not be caught by any other gate) | — |
| **Writing miri-runnable tests** | **Off-by-one in unsafe code path (1/6, highest severity)** | — |
| Miri (the actual tool) | **Nothing in this POC** — either couldn't run (shared-memory-server) or passed cleanly (metadata.rs) | — |
| Semver check | Nothing | Baseline established |

**Four of six findings (#3, #4, #5, #6) would have been missed by every gate that existed in dora prior to this POC.** That is the core validation of the agentic QA strategy: the new gates catch things the old gates couldn't.

### 10a.3 Meta-finding: "infrastructure as forcing function"

The single highest-severity bug (#6, `metadata::from_array` off-by-one) was **not** found by running a tool. It was found by **reading the code while preparing to use a tool**.

Specifically: I sat down to run miri on `metadata.rs`'s unsafe pointer arithmetic. To do that I needed unit tests. While writing the unit tests, I read `from_array` carefully enough to spot the bug by eye. Miri then ran cleanly on the fix. The tool didn't catch the bug; the discipline of preparing the tool did.

**Lesson for the strategy**: new gates are valuable even when they don't directly fire. They force close-reading of code that would otherwise never be close-read. This is an argument for a broader rollout, not a narrower one — because the "reading discipline" cost is per-gate-target, not per-gate-type.

**Implication**: the section on Tier 2 miri targets should expand beyond "run miri" to "write the miri-required tests." The tests themselves are the most valuable output, miri runs are the gate.

### 10a.4 Meta-finding: `shared-memory-server` is a critical QA blind spot

**Corrects Section 6.3 of this doc**, which recommended running miri on `shared-memory-server`.

Miri cannot run `shared-memory-server` tests. Every test in that crate calls `ShmemConf::create` which invokes libc's `shm_open`, and miri's foreign function support does not cover POSIX shared memory syscalls on any platform.

This matters because:

1. `shared-memory-server` contains **30 unsafe blocks** — the single largest concentration of `unsafe` in the workspace.
2. It handles **every local node↔daemon message** (the 4 control channels per node plus dynamic data regions for messages ≥ 4KB).
3. The 2026-03-21 audit found 3 memory-safety issues in exactly this area (shmem OOB read, unsound `Sync` impl, ARM data race). Those were caught by manual audit, not automated tools.
4. None of our current tooling covers it:
   - Miri: can't run (FFI)
   - Property testing: no pure-Rust entry points to target
   - Fuzzing: same reason
   - Coverage: only covered by integration tests that spin up real shmem
   - Mutation testing: would mutate unsafe code but tests take 60+ seconds each (real shmem lifecycle)

**Options** (none chosen yet, this is for discussion):

- **Option A:** Refactor `shared-memory-server` to have a pure-Rust core (all pointer arithmetic) and a libc wrapper (shm_open/mmap). The pure-Rust core gets miri + proptest + fuzz; the wrapper is thin. Moderate work.
- **Option B:** Adopt Zenoh's native shared memory feature (see `plan-zenoh-shared-memory.md` — upstream dora already did this in `dora-rs/adora#1378`). Deletes 660 lines and ~11 unsafe blocks from `channel.rs`, removes the pinned `raw_sync_2 =0.1.5` fork dep, aligns with upstream. **This is also the right thing to do as part of the dora 1.0 consolidation.**
- **Option C:** Add a mock-shmem backend conditionally compiled for miri targets. Complex, maintenance burden.

**Recommendation: Option B, sequenced as part of dora 1.0 consolidation.** Fold the zenoh-SHM migration into the consolidation merge. The 30 unsafe blocks stop being an dora problem because they stop being code. See `plan-dora-1.0-consolidation.md` for the amendment.

### 10a.5 Meta-finding: proptest strategies need scope discipline

The `ws_protocol` property tests initially failed on two things:

1. **f64 precision near the edge** — arbitrary `f64` values like `-5.78e+120` lose 1 ULP through the JSON Number roundtrip. Not an dora bug, a JSON limitation. The proptest strategy was too broad; it was testing a property of JSON itself, not a property of dora's serialization.

2. **`Some(Value::Null)` asymmetry** — a real bug in dora that the broad strategy uncovered.

**Lesson**: broad proptest strategies find a mix of real bugs and "properties of the underlying platform." Triaging which is which is a skill. The first reaction to a proptest failure should be:
- Reproduce manually
- Ask "is this a property of my code, or a property of what my code depends on?"
- If the latter: constrain the strategy, document why
- If the former: it's a real finding, fix or document as invariant

**Action item for the doc**: Section 6.1 (property-based testing) should add a paragraph on "scope discipline" — writing strategies that test *your* invariants, not the platform's.

### 10a.6 Meta-finding: the unwrap-budget script has a false-positive mode

The `scripts/qa/unwrap-budget.sh` script counts `.unwrap()` inside `#[cfg(test)] mod tests {}` blocks that live inline in source files (as opposed to the `tests/` directory, which is correctly excluded). During the POC the budget bumped 622 → 638 → 643 from adding test-code unwraps — no production code changed.

**Current state**: documented as a KNOWN LIMITATION in the script header.

**Proper fix queued**: parse Rust with `syn` or `ast-grep` to count only non-test scopes. Moderate effort.

### 10a.7 Amendments to this document

Based on the POC, the following sections of this doc should be updated:

| Section | Amendment |
|---|---|
| 5.2 Mutation testing | Add: "Initial runs typically find 60-70% missed mutations. Triage into real test gaps (most), tautological tests (some), and equivalent mutants (few, 3-10%). Document equivalent mutants with explicit `exclude_re` entries — see `.cargo/mutants.toml` for the pattern." |
| 5.2 Mutation testing | Add: "`exclude_re` regexes should match on the mutation name only, never pin to line:column numbers — fragility lesson from commit `3ff3785`." |
| 6.1 Property-based testing | Add: "Scope discipline is critical. Constrain strategies to inputs that test *your* invariants, not properties of the underlying serialization format. f64 values near precision limits do not round-trip exactly through JSON; filter them out of strategies that test serde roundtrips." |
| 6.3 Miri | **Replace** the "miri on `shared-memory-server`" recommendation with: "`shared-memory-server` is not miri-runnable (FFI dependency). Target `metadata.rs`, `arrow-convert`, and any other pure-Rust crate with unsafe blocks instead. Prepare the miri rollout by first writing focused unit tests for each unsafe function — this step has historically caught more bugs than miri itself." |
| 6.3 Miri | Add: "Preferred long-term solution for shared-memory-server coverage: adopt Zenoh SHM (see `plan-zenoh-shared-memory.md`). This is sequenced into the dora 1.0 consolidation." |
| 8 POC plan | Add actual POC results section after this plan is fully executed. |

These amendments are **to-do** and should be applied in a follow-up commit.

---

## 11. Metrics and KPIs

### 11.1 PR-level (agent feedback loop)

- Time from PR open to Tier 1 green (target: <20 min)
- Number of Tier 1 iterations per PR (target: ≤3)
- Adversarial review issues raised per PR (target: ≤2; higher = author needs guidance)
- Coverage delta per PR (target: non-negative)
- Mutation score delta per PR (target: non-negative)

### 11.2 Nightly (Tier 2)

- Nightly Tier 2 pass rate (target: >95%)
- Time to green on a broken nightly (target: <24 hours)
- New fuzz crashes found per week (target: 0)
- New miri UB findings per week (target: 0)

### 11.3 Release (Tier 3)

- External audit critical/high count at release (target: 0)
- Dogfood week unexpected restart count (target: 0)
- p99 latency regression vs. previous release (target: <5%)
- Memory growth during dogfood (target: flat after 1-hour warmup)

### 11.4 Long-term agentic quality

- Bug reports per release in the first 30 days (rolling trend)
- Coverage trend over 3 months
- Mutation score trend over 3 months
- Unwrap budget trend over 3 months
- Unsafe block count trend over 3 months
- Architectural drift alerts per month (from fitness tests)

---

## 12. Appendix A: Tool reference

| Tool | Purpose | Install | Docs |
|---|---|---|---|
| `cargo-llvm-cov` | Code coverage | `cargo install cargo-llvm-cov` | https://github.com/taiki-e/cargo-llvm-cov |
| `cargo-mutants` | Mutation testing | `cargo install cargo-mutants` | https://mutants.rs |
| `cargo-audit` | CVE check | `cargo install cargo-audit` | https://docs.rs/cargo-audit |
| `cargo-deny` | License + dep policy | `cargo install cargo-deny` | https://embarkstudios.github.io/cargo-deny/ |
| `cargo-semver-checks` | SemVer breakage | `cargo install cargo-semver-checks` | https://github.com/obi1kenobi/cargo-semver-checks |
| `cargo-fuzz` | libFuzzer wrapper | `cargo install cargo-fuzz` | https://rust-fuzz.github.io/book/ |
| `proptest` | Property testing | (dev-dep) `proptest = "1.5"` | https://docs.rs/proptest |
| `miri` | UB interpreter | `rustup +nightly component add miri` | https://github.com/rust-lang/miri |
| `madsim` | Deterministic distsim | (dev-dep) `madsim = "0.2"` | https://docs.rs/madsim |
| `cargo-public-api` | API surface diff | `cargo install cargo-public-api` | https://github.com/Enselic/cargo-public-api |
| `diff-cover` | Coverage diff gate | `pip install diff-cover` | https://github.com/Bachmann1234/diff_cover |
| `codecov` | Coverage dashboard | (GitHub Action) | https://codecov.io |

---

## 13. Appendix B: Minimal CI workflow template

Complete workflow file for Tier 1 + basic Tier 2. Copy to `.github/workflows/qa.yml`.

```yaml
name: QA

on:
  pull_request:
    branches: [main]
  push:
    branches: [main]
  schedule:
    - cron: '0 2 * * *'  # Tier 2 nightly

env:
  CARGO_TERM_COLOR: always
  RUST_BACKTRACE: 1

jobs:
  # ============================================================
  # Tier 1 — runs on every PR
  # ============================================================

  coverage:
    if: github.event_name == 'pull_request'
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
        with:
          fetch-depth: 0
      - uses: dtolnay/rust-toolchain@master
        with:
          toolchain: 1.92
          components: llvm-tools-preview
      - uses: Swatinem/rust-cache@v2
      - uses: taiki-e/install-action@cargo-llvm-cov
      - run: |
          cargo llvm-cov --workspace \
            --exclude dora-node-api-python \
            --exclude dora-operator-api-python \
            --exclude dora-ros2-bridge-python \
            --lcov --output-path lcov.info
      - uses: codecov/codecov-action@v4
        with:
          files: lcov.info
      - run: |
          pip install diff-cover
          diff-cover lcov.info --compare-branch=origin/main --fail-under=80

  mutants:
    if: github.event_name == 'pull_request'
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
        with:
          fetch-depth: 0
      - uses: dtolnay/rust-toolchain@master
      - uses: Swatinem/rust-cache@v2
      - uses: taiki-e/install-action@v2
        with:
          tool: cargo-mutants
      - run: |
          cargo mutants --in-diff origin/main \
            --package dora-core --package dora-daemon \
            --package dora-coordinator --package dora-message \
            --timeout 120 --jobs 4

  supply-chain:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: taiki-e/install-action@v2
        with:
          tool: cargo-audit,cargo-deny
      - run: cargo audit --deny warnings
      - run: cargo deny check

  unwrap-budget:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - run: bash scripts/check-unwrap-budget.sh

  adversarial-review:
    if: github.event_name == 'pull_request'
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
        with:
          fetch-depth: 0
      - run: |
          git diff origin/main...HEAD > /tmp/diff.patch
          python scripts/adversarial-review.py /tmp/diff.patch \
            --model claude-opus-4-6 \
            --post-comment \
            --pr ${{ github.event.pull_request.number }}
        env:
          ANTHROPIC_API_KEY: ${{ secrets.ANTHROPIC_API_KEY }}
          GITHUB_TOKEN: ${{ secrets.GITHUB_TOKEN }}

  semver:
    if: github.event_name == 'pull_request'
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
        with:
          fetch-depth: 0
      - uses: dtolnay/rust-toolchain@master
      - uses: taiki-e/install-action@v2
        with:
          tool: cargo-semver-checks
      - run: |
          for crate in dora-node-api dora-operator-api dora-core dora-message; do
            cargo semver-checks check-release -p $crate || true
          done

  # ============================================================
  # Tier 2 — runs nightly
  # ============================================================

  fuzz:
    if: github.event_name == 'schedule'
    runs-on: ubuntu-latest
    timeout-minutes: 90
    steps:
      - uses: actions/checkout@v4
      - uses: dtolnay/rust-toolchain@nightly
      - uses: taiki-e/install-action@v2
        with:
          tool: cargo-fuzz
      - uses: actions/cache@v4
        with:
          path: libraries/core/fuzz/corpus
          key: fuzz-corpus-${{ github.sha }}
          restore-keys: fuzz-corpus-
      - run: |
          for target in yaml_descriptor ws_protocol topic_data_frame \
                        shared_memory_frame recording_format; do
            cd libraries/core
            timeout 600 cargo fuzz run $target -- -max_total_time=600 || exit 1
            cd ../..
          done

  miri:
    if: github.event_name == 'schedule'
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: dtolnay/rust-toolchain@master
        with:
          toolchain: nightly
          components: miri
      - run: cargo miri test -p shared-memory-server
      - run: cargo miri test -p dora-arrow-convert
      - run: cargo miri test -p dora-message

  mutants-full:
    if: github.event_name == 'schedule' && github.event.schedule == '0 2 * * 0'
    runs-on: ubuntu-latest
    timeout-minutes: 480
    steps:
      - uses: actions/checkout@v4
      - uses: dtolnay/rust-toolchain@master
      - uses: Swatinem/rust-cache@v2
      - uses: taiki-e/install-action@v2
        with:
          tool: cargo-mutants
      - run: |
          cargo mutants --package dora-core --package dora-daemon \
                        --package dora-coordinator --package dora-message \
                        --package dora-coordinator-store \
                        --package shared-memory-server \
                        --jobs 4 --timeout 120 --json > mutants.json
```

---

## 14. Appendix C: Adversarial review prompt (full)

Save as `scripts/adversarial-review-prompt.txt`. The Python wrapper reads this and injects the diff.

```
You are a senior staff engineer performing an adversarial pre-merge review of
a pull request. The PR was likely authored by an AI coding agent. Your task
is to find the issues the agent probably missed.

You have three rules:

1. Be specific. For every issue, cite file:line and explain concretely.
2. Be skeptical, not inventive. If you can't find real issues, say so. Don't
   fabricate problems to justify your existence.
3. Prioritize: list your top 3 concerns first; everything else is "also".

The common AI-agent failure modes you are looking for:

**Tautological tests.** Tests that mirror the implementation instead of
asserting behavior. Example: a test that calls the function being tested with
inputs the function was written for, then asserts the output is what the
function returned. The test is guaranteed to pass regardless of the
function's correctness.

**Unreachable defensive code.** Error paths, validation, fallbacks that
cannot be triggered by any realistic input. Ask: "what input would cause
this branch to execute?" If you can't answer, flag it.

**Invariant violations.** A value is constructed that breaks its own type's
invariant. Example: `ParamReplaySummary { attempted: 0, failed: 1 }` where
`failed > attempted` is impossible for the function itself.

**Silent error swallowing.** `.ok()`, `let _ = ...`, `if let Err(_) = ...`
that loses information. Ask: "how would we debug this when it breaks?"

**New unwrap or expect in non-test code.** Each is a potential panic at
runtime. Ask: "is this actually infallible, or just conveniently typed?"

**Concurrent bugs.** Lock ordering, shared state accessed without
synchronization, async tasks that outlive their intended scope, mutex
acquired but not released in an error path.

**Breaking API changes not flagged.** Public signatures changed, enum
variants added (without `#[non_exhaustive]`), trait methods added without
defaults, default values changed.

**Scope creep.** Changes in the PR that have nothing to do with the PR
title or description. A fix PR that also "improves" unrelated code is
a red flag.

**Missing tests for new branches.** New `if` statements or `match` arms
that have no corresponding test case.

**Missing tests for the disconnected case.** If the code now handles a
"not connected," "not found," "None," or "empty" branch, verify a test
covers it.

**Poorly-scoped mocks.** Mocks of the database, network, or filesystem
that make the test pass without verifying real behavior. Ask: "if I
replaced this mock with the real thing, would the test still pass?"

Output format:

## Top concerns

1. <file:line — one-sentence headline>
   <2-4 sentences explaining the concern>

2. <file:line — one-sentence headline>
   <2-4 sentences>

3. <file:line — one-sentence headline>
   <2-4 sentences>

## Also

- <file:line — brief note>
- <file:line — brief note>

## No-issue areas

(1-2 sentences noting areas you reviewed and found clean)

---

If you find nothing of concern, say exactly: "Reviewed the full diff;
no issues found. Specifically checked for tautological tests, unreachable
defensive code, invariant violations, silent error swallowing, new
unwraps, concurrency, breaking API changes, scope creep, missing test
branches, and over-mocked tests."

The diff follows.
```

---

## 15. Change log

| Date | Author | Change |
|---|---|---|
| 2026-04-07 | heyong4725 + AI | Initial draft |

---

## 16. Related documents

- [`plan-dora-1.0-consolidation.md`](plan-dora-1.0-consolidation.md) — The consolidation plan this QA strategy supports. Q1-minimum items from this doc must land before the dora 1.0 release.
- [`audit-report-2026-03-21.md`](audit-report-2026-03-21.md) — The one-shot audit that motivates the cadenced audit policy in T3.1.
- [`testing-guide.md`](testing-guide.md) — Current test infrastructure; superseded in parts by this doc.
- [`architecture.md`](architecture.md) — System architecture reference.
