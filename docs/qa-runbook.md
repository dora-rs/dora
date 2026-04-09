# QA Runbook

**Audience**: adora developers (human or agent) running QA locally.
**Purpose**: tell you which command to run, how to read the output, and what to do when something fails.
**Deep dives**: see [`plan-agentic-qa-strategy.md`](plan-agentic-qa-strategy.md) for the strategy and rationale; this document is the operational reference.

---

## 1. TL;DR

```bash
# Before every commit (~15 seconds)
make qa-fast

# Before every push (~5-10 minutes)
make qa-full

# Before a release or a focused mutation audit (~1+ hour)
make qa-tier1
```

If you only remember one command: **`make qa-fast`**.

First-time setup on a fresh clone:

```bash
make qa-install     # installs cargo-audit, cargo-deny, cargo-llvm-cov, cargo-mutants, cargo-semver-checks
pip install diff-cover   # optional; enables diff coverage gate
rustup component add miri --toolchain nightly   # optional; for unsafe-code analysis
```

---

## 2. What each target runs

| Target | Runs | Budget | When to use |
|---|---|---|---|
| `make qa-fast` | fmt + clippy + audit + unwrap-budget | ~15 s | Pre-commit |
| `make qa-full` | `qa-fast` + full test suite + coverage | ~5-10 min | Pre-push |
| `make qa-tier1` | `qa-full` + mutation testing on diff + semver | ~1-2 hrs | Pre-release or focused audit |
| `make qa-fmt` | `cargo fmt --all -- --check` | ~2 s | Spot-check |
| `make qa-clippy` | `cargo clippy --all -- -D warnings` (excluding Python) | ~1 min | After mechanical edits |
| `make qa-audit` | `cargo audit` + `cargo deny check` | ~10 s | After bumping deps |
| `make qa-unwrap` | count `.unwrap()` / `.expect(` in production code | ~2 s | After adding unwraps |
| `make qa-test` | `cargo test --all` (excluding Python) | ~3-5 min | After code changes |
| `make qa-coverage` | `cargo llvm-cov` (writes `lcov.info`) | ~5 min | To see coverage locally |
| `make qa-mutants` | `cargo mutants --in-diff origin/main` on critical crates | ~5-30 min | To verify tests actually detect bugs |
| `make qa-semver` | `cargo semver-checks` vs last tag | ~1-2 min | Before bumping published crate versions |

All targets call scripts under `scripts/qa/`. The scripts are the source of truth — if something looks wrong, read the script.

---

## 3. Reading failures and fixing them

### 3.1 `fmt` failed

**Cause**: your edits don't match `rustfmt`.
**Fix**: `cargo fmt --all`. Re-run `make qa-fast`.

### 3.2 `clippy` failed

**Cause**: a clippy lint fires on your code. Common ones in adora:
- `collapsible_if` — nested `if let`s can use `&&` chain syntax (Rust 2024).
- `let-and-return`
- Needless `.clone()`

**Fix**:
1. Read the error — clippy points at file:line.
2. Try `cargo clippy --fix --allow-dirty --allow-staged` for mechanical fixes.
3. Re-run. If it still fails, address manually.
4. **Do not** add `#[allow(...)]` without a comment explaining why.

### 3.3 `audit` failed (cargo-audit)

**Cause**: one of our transitive dependencies has a new RustSec advisory.

**Step 1 — classify the advisory:**

- **Real vulnerability (Severity: Medium/High/Critical)**: must fix before merging. Follow step 2.
- **Unmaintained crate warning** (most common): can be waived if fixing is expensive. Follow step 3.

**Step 2 — fix a real vulnerability:**

```bash
# Often cargo update alone resolves it:
cargo update -p <crate> --precise <fixed-version>

# If the vulnerable crate is a transitive dep of something else that pins it,
# you may need to upgrade the parent crate. Read the "Dependency tree" in
# the cargo-audit output to find the path.
```

Verify: `make qa-audit` returns exit 0.

**Step 3 — waive an unmaintained warning:**

Open `deny.toml`, add the advisory ID to `advisories.ignore` with a comment and review date:

```toml
[advisories]
ignore = [
    "RUSTSEC-2025-XXXX",  # foo-crate unmaintained; transitive via bar; review 2026-10
]
```

Run `make qa-audit` again — should pass.

### 3.4 `unwrap-budget` failed

**Cause**: your changes added a `.unwrap()` or `.expect(` in non-test code.

**Step 1 — identify the new unwraps:**

```bash
rg --type rust '\.unwrap\(\)|\.expect\(' libraries/ binaries/ apis/ \
  -g '!**/tests/**' -g '!**/benches/**' -g '!**/examples/**'
```

Compare against `git diff` to see which ones are yours.

**Step 2 — decide:**

- **Replace with proper error handling**: the preferred fix. Use `?` on `Result`, or `ok_or()` / `ok_or_else()` on `Option`.
- **Replace with `.expect("reason")`** if genuinely infallible: still counts toward the budget but at least documents the invariant.
- **Bump the budget**: acceptable only when (a) the unwrap is genuinely infallible by construction (e.g., writing to a `String`) AND (b) you justify it in the commit message. Update `.unwrap-budget` in the same commit.

Note: the budget ratchet is intentionally asymmetric. You can reduce the number freely; any increase needs justification.

### 3.5 `test` failed

**Cause**: you broke a test.

**Fix**: run the failing test in isolation for clearer output:

```bash
cargo test -p <crate> <test_name> -- --nocapture
```

If the test was wrong and the code is right, fix the test. If the code was wrong, fix the code. Don't fix the test to match broken code.

### 3.6 `coverage` (soft) flagged

**Cause**: the diff coverage gate (if running on a PR) found less than 70% of your new/changed lines are covered by tests.

**Fix**: add tests that exercise the new code paths. The gate is soft on `main` — it only fails on PRs. Locally you can see the uncovered lines via:

```bash
make qa-coverage
open target/llvm-cov/html/index.html   # if you also run `cargo llvm-cov --html`
```

### 3.7 `mutation` escaped

**Cause**: `cargo-mutants` found a mutation that no test detected — meaning your tests are incomplete for the mutated code path.

**Step 1 — understand what the mutation is:**

The output shows lines like:
```
libraries/core/src/types.rs:168:18: replace == with != in types_match
```

Read: "cargo-mutants changed `==` to `!=` on line 168, re-ran the tests, and they all still passed."

**Step 2 — write a test that catches it:**

Construct an input where the mutated version produces a different output from the original. For the example above: a test that asserts `types_match("a", "b") == false` would catch the `==→!=` mutation (unmutated returns false, mutated returns true).

**Step 3 — if it's a genuine equivalent mutant**: document it in `.cargo/mutants.toml` with a detailed comment explaining why the mutation produces semantically identical behavior. See the `types_match` `||→&&` example already in the file.

**Do not** waive mutations just to make the gate pass. The point of the gate is to surface weak tests.

### 3.8 `semver` (soft) flagged

**Cause**: `cargo-semver-checks` found a breaking change in a publishable crate's public API since the last tag.

**Fix**:
- If the change is intentional: make sure the crate's version is bumped to a new major or minor (per SemVer rules) before release.
- If unintentional: revert the breaking change.

The gate is soft during 0.x development — it only warns. It becomes a hard gate after the 1.0 release per `plan-dora-1.0-consolidation.md`.

---

## 4. Running the adversarial LLM review (local only today)

**Prerequisites**: either `codex` (OpenAI Codex CLI) or `claude` (Claude Code CLI) installed and logged in.

```bash
# Auto: detects backend, diffs vs origin/main
./scripts/qa/adversarial.sh

# Explicit backend
./scripts/qa/adversarial.sh --backend claude
./scripts/qa/adversarial.sh --backend codex

# Review a specific diff file
./scripts/qa/adversarial.sh --diff my-patch.diff

# Review against a different base
./scripts/qa/adversarial.sh --base HEAD~5
```

Output goes to stdout and `/tmp/adversarial-review-<short-sha>.md`. Read the review, respond to each flagged issue.

The prompt template is at `scripts/qa/adversarial-prompt.md`. Tune it if the review is too noisy or too lax.

**CI integration** is pending — requires `ANTHROPIC_API_KEY` or equivalent set as a repo secret.

---

## 5. Running miri on the unsafe hotspots

**Prerequisites**:

```bash
rustup component add miri --toolchain nightly
```

**Run**:

```bash
# metadata.rs — known-good target with focused unit tests
cargo +nightly miri test -p adora-core metadata::tests

# Add more targets as they gain focused unit tests:
# cargo +nightly miri test -p adora-coordinator-store
# cargo +nightly miri test -p adora-arrow-convert
```

**Do NOT** run miri on `shared-memory-server` — its tests call libc's `shm_open` which miri does not support. Every test aborts with "unsupported operation". See `plan-agentic-qa-strategy.md` Section T2.3 for the explanation and the long-term fix (Zenoh SHM migration).

---

## 6. Running property tests

Proptest strategies live inside `#[cfg(test)] mod tests` in the target source files. They run automatically under `cargo test`.

```bash
cargo test -p adora-message ws_protocol::tests::prop_
```

To increase the case count for a focused hunt:

```bash
PROPTEST_CASES=20000 cargo test -p adora-message ws_protocol::tests::prop_
```

Failed proptest cases are saved in `libraries/message/proptest-regressions/` and **should be committed** to source control — they re-run as fast regression checks on every test run.

---

## 7. Running focused mutation tests (when investigating a file)

```bash
# Mutation test a single file
cargo mutants --package <pkg> --file <path/to/file.rs> --jobs 4 --timeout 120

# List mutations without running (fast)
cargo mutants --package <pkg> --file <path/to/file.rs> --list

# Only mutations in the current diff (fast, for PR checks)
cargo mutants --in-diff origin/main --package <pkg>
```

**Tip**: mutation runs are expensive. When investigating a specific bug hypothesis, scope to a single file.

**Tip**: the full critical-crate chain takes about 17 hours (`adora-core`, `adora-message`, `adora-coordinator-store`, `adora-coordinator`, `adora-daemon`) with workspace-scoped tests. Budget accordingly.

---

## 8. The unwrap budget ratchet explained

`.unwrap-budget` contains a single integer: the maximum allowed count of `.unwrap()` and `.expect(` in production code.

- The number can only go **down** over time.
- Your PR's unwrap count must be ≤ the stored budget.
- If your PR **reduces** the count, commit the smaller number in the same PR.
- If your PR **increases** the count, either (a) fix the new unwraps or (b) bump the number **with justification in the commit message**.

The script counts everything under `libraries/`, `binaries/`, `apis/` **except**:
- Files inside `tests/`, `benches/`, `examples/` directories
- Files named `tests.rs` (submodule test files)
- Lines after the first `#[cfg(test)]` in any source file

See the KNOWN LIMITATION notes in `scripts/qa/unwrap-budget.sh` for edge cases.

---

## 9. Where to look when something is weird

| Symptom | Look here |
|---|---|
| Local run differs from CI | `.github/workflows/ci.yml` vs `scripts/qa/*.sh` — they should call the same commands |
| Tool not installed | `make qa-install` |
| CI cache stale | Clear cache in GitHub Actions UI, or bump `Swatinem/rust-cache` key |
| Mutation score looks wrong | Check `.cargo/mutants.toml` `test_workspace = true` is set |
| `coverage.sh` hangs | Usually one test is stuck — run with `cargo test` directly to find it |
| Unwrap budget counts test code | The script should exclude test code; if it doesn't, report a bug in the script |

---

## 10. Adding a new gate

If you want to add a new QA check:

1. Write the check as a shell script under `scripts/qa/<name>.sh`.
2. Make it executable, runnable standalone, and fail-fast.
3. Add a target to `Makefile` (`qa-<name>`).
4. Add it to `scripts/qa/all.sh` in the appropriate tier (fast / full / tier1).
5. Add a CI job to `.github/workflows/ci.yml` that calls `make qa-<name>`.
6. Document it in this runbook (Section 3).

---

## 11. Related documents

- [`qa-followups.md`](qa-followups.md) — **open items tracker**: everything the POC deferred, organized by effort and trigger. Check here for "what's left to do"
- [`plan-agentic-qa-strategy.md`](plan-agentic-qa-strategy.md) — the full strategy, including the three-tier model, case studies, and meta-findings
- [`qa-baseline-2026-04-07.md`](qa-baseline-2026-04-07.md) — metrics snapshot as of the POC session
- [`plan-dogfood-campaign.md`](plan-dogfood-campaign.md) — pre-release dogfood campaign spec
- [`plan-fault-injection.md`](plan-fault-injection.md) — chaos scenarios queued for implementation
- [`qa-poc-report-2026-04-09.md`](qa-poc-report-2026-04-09.md) — comprehensive POC report for outside readers
