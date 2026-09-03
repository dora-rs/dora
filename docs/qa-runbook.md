# QA Runbook

**Audience**: dora developers (human or agent) running QA locally.
**Purpose**: tell you which command to run, how to read the output, and what to do when something fails.
**Deep dives**: see [`plan-agentic-qa-strategy.md`](plan-agentic-qa-strategy.md) for the strategy and rationale; this document is the operational reference.
**Coverage by capability**: see [`testing-capabilities.md`](testing-capabilities.md) (#1633) when you need to know *which* tests cover a feature before touching it.
**Validation bar by change class**: see [`agentic-qa-policy.md`](agentic-qa-policy.md) (#1634) to know *how much* validation a change needs.

---

## 1. TL;DR

```bash
# Before every commit (~15 seconds)
make qa-fast

# Before every push (~5-10 minutes)
make qa-full

# Target Tier 1 gate -- stronger than today's CI (~15 minutes)
make qa-deep

# Overnight run on a beefy machine -- Tier 2 equivalent (~30-60 min)
make qa-nightly

# Before tagging a release
make qa-release-gate

# Deliberate test-quality audit (NOT every nightly; takes 10-18 hrs)
make qa-mutation-audit

# Run all smoke-eligible example dataflows end-to-end (~15-20 min, orthogonal to ladder)
# Skips CUDA/ROS2/webcam/C++/interactive examples -- run `scripts/smoke-all.sh -h` for the SKIP list
make qa-examples
# or scope it: make qa-examples ARGS="--rust-only"
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
| `make qa-fast` | fmt + clippy + audit + unwrap-budget + secret-files + typos + publish-graph + package-includes + breaking-changes | ~15 s | Pre-commit |
| `make qa-full` | `qa-fast` + full test suite + coverage | ~5-10 min | Pre-push |
| `make qa-deep` | `qa-full` + mutation testing on diff + semver | ~15 min | Target Tier 1 local gate (stronger than today's CI: adds coverage, adversarial, mutants, semver) |
| `make qa-tier1` | alias for `qa-deep` | — | Back-compat; prefer `qa-deep` |
| `make qa-nightly` | `qa-deep` + proptest@1000 + miri (if installed) + example-smoke (in scratch venv with `-e apis/python/node`) + hub-smoke + ci-nightly-jobs | ~3-4 hours | Full parity with `.github/workflows/nightly.yml` after the #1716 rebalance: **19 test jobs total**. example-smoke covers the **4 example-backed** GHA jobs (smoke-suite, log-sinks, service-action, streaming); hub-smoke covers the **Hub e2e** job (tests/hub-smoke.rs); `scripts/qa/ci-nightly-jobs.sh` drives the **14 remaining** with platform-aware dispatch (record-replay, cluster-smoke, topic-and-top, cpu-affinity [Linux], redb-backend, daemon-reconnect [Linux], state-reconstruction, test-cross-platform [macOS+Windows], examples, cli-tests, bench-example, cross-check, ros2-bridge [Linux+ROS2], msrv). Requires **both `uv` and Python 3.12** — both preflighted; fails fast with a specific install hint for whichever is missing (`curl -LsSf https://astral.sh/uv/install.sh \| sh` for uv, `uv python install 3.12` for the interpreter). example-smoke installs workspace Python bindings into the scratch venv to match the GHA Python setup (avoids PyPI drift, #1710). Green local run on platform X predicts a green CI nightly for platform X's jobs; jobs that can't run on the dev's OS SKIP cleanly. Does NOT include full-repo mutation testing (see `qa-mutation-audit`). |
| `make qa-release-gate` | `qa-deep` + semver | ~15 min | The automatable subset of Tier 3. Non-automatable: security audit + dogfood + migration validation (see strategy doc §7) |
| `make qa-mutation-audit` | `cargo-mutants --full` on 6 critical crates | ~10-18 hrs | Deliberate test-quality audit, not every nightly |
| `make qa-examples` | `scripts/smoke-all.sh` -- all smoke-eligible example dataflows end-to-end (skips CUDA/ROS2/webcam/C++/interactive) | ~15-20 min | When you want actual dataflows exercised. Orthogonal to ladder -- qa-fast/full/deep all `--exclude dora-examples`. Pass `ARGS="--rust-only"` etc. |
| `make qa-fmt` | `cargo fmt --all -- --check` | ~2 s | Spot-check |
| `make qa-clippy` | `cargo clippy --all --all-targets -- -D warnings` (excluding Python) | ~1 min | After mechanical edits |
| `make qa-audit` | `cargo audit` + `cargo deny check` | ~10 s | After bumping deps |
| `make qa-unwrap` | count `.unwrap()` / `.expect(` in production code | ~2 s | After adding unwraps |
| `make qa-test` | `cargo test --all` (excluding Python) | ~3-5 min | After code changes |
| `make qa-coverage` | `cargo llvm-cov` (writes `lcov.info`) | ~5 min | To see coverage locally |
| `make qa-mutants` | `cargo mutants --in-diff origin/main` on critical crates | ~5-30 min | To verify tests actually detect bugs |
| `make qa-semver` | `cargo semver-checks` vs last tag | ~1-2 min | Before bumping published crate versions |
| `make qa-breaking` | every surface dora 1.x freezes, vs the last release tag | ~2 s (`ARGS="--fast"`) / ~2-5 min (full) | The `--fast` half runs in `qa-fast` and in PR CI; run the full one when you touched the CLI, the descriptor, `dora-message`, or a node API |
| `make qa-breaking-update` | re-record the CLI snapshot and the JSON schemas | ~1-2 min | After *adding* a command, flag or descriptor field — commit the diff |

All targets call scripts under `scripts/qa/`. The scripts are the source of truth — if something looks wrong, read the script.

---

## 3. Reading failures and fixing them

### 3.1 `fmt` failed

**Cause**: your edits don't match `rustfmt`.
**Fix**: `cargo fmt --all`. Re-run `make qa-fast`.

### 3.2 `clippy` failed

**Cause**: a clippy lint fires on your code. Common ones in dora:
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

### 3.5 `secret-files` failed

**Cause**: git is tracking a file that either matches a `.gitignore` rule or carries a name that claims to hold a credential.

**If it is a real credential**: deleting the file is not the fix. The value stays readable in history (`git show <commit>^:<path>`) and in every clone and fork that already pulled — which is why `.adora-token` (#2194) needed rotation, not just removal. Rotate the credential first, then remove the file.

**If it is a fixture or a template**: rename it so the name no longer claims to be live — `.env.example`, `test-key.pem.template`. The gate anchors its patterns at the end of the name, so those suffixed forms pass.

**If the `.gitignore` half is what flagged it**: a tracked file matching an ignore rule is worth a human look even when it holds no secret. It means the file was force-added or arrived through an import that bypassed the ignore rules — exactly how `.adora-token` got in.

Reproduce locally with `make qa-secret-files`; the rationale is documented at the top of [`scripts/qa/secret-files.sh`](../scripts/qa/secret-files.sh).

### 3.6 `publish-graph` failed

**Cause**: a change to the publish graph that `cargo publish` would reject — but only at release time, once crates.io already holds whatever the release uploaded before the failure. The gate reports one of three things.

**"X depends on Y, which is `publish = false`"**: cargo resolves every dependency of a published crate against the registry, so Y has to be on crates.io for X to publish. Either publish Y (drop `publish = false`, add it to both publish lists, and place it in a tier in [`api-rust.md`](api-rust.md#stability-scope-at-10)), or stop X from naming it. Optional dependencies are not an escape: they are in the published manifest whether or not the feature is on. That is #3304, and it is why `dora-tensor-pool` is published.

**"which the publish lists publish after it" / "missing from the publish lists"**: the lists are consumed in order, and a crate cannot publish before its dependencies exist in the index. Move the dependency earlier, or add it.

Reproduce locally with `make qa-publish-graph`; the rules and what each protects are documented at the top of [`scripts/qa/publish-graph.sh`](../scripts/qa/publish-graph.sh).

### 3.7 `package-includes` failed

**Cause**: a publishable crate reads a file at build time that its published `.crate` would not contain. `cargo package` ships the git-tracked files under one crate directory and nothing else, so the workspace build stays green and the failure lands on whoever runs `cargo install` — after the version is on crates.io and can no longer be replaced.

**"resolves to X, outside crate Y"**: the path leaves the crate directory, usually a `../` into a sibling crate. Keep a crate-local copy of the file and include that; if two crates need the same file, add a test that keeps the copies identical, as [`tests/cmake-template-sync.rs`](../tests/cmake-template-sync.rs) does for the cmake templates. That test lives in `dora-examples`, which every bulk `cargo test` excludes, so no `make qa-*` target runs it — drift in the copies is caught by the `contract-tests` CI job, or locally by naming it: `cargo test -p dora-examples --test cmake-template-sync`. This is #3400: `dora-operator-api-c/build.rs` read `../node/cmake/*.cmake.in`, and `cargo install dora-cli` failed for every 1.0.0 user.

**"which git does not track"**: the path is inside the crate but the file is generated or ignored, so packaging drops it. Commit the file, or have the build script write it into `OUT_DIR` and include it from there.

Reproduce locally with `make qa-package-includes`, and confirm a fix end to end with `cargo package -p <crate>` — that builds the crate from its own tarball, which is exactly what the gate approximates statically.

### 3.8 `test` failed

**Cause**: you broke a test.

**Fix**: run the failing test in isolation for clearer output:

```bash
cargo test -p <crate> <test_name> -- --nocapture
```

If the test was wrong and the code is right, fix the test. If the code was wrong, fix the code. Don't fix the test to match broken code.

### 3.9 `coverage` (soft) flagged

**Cause**: the diff coverage gate (if running on a PR) found less than 70% of your new/changed lines are covered by tests.

**Fix**: add tests that exercise the new code paths. The gate is soft on `main` — it only fails on PRs. Locally you can see the uncovered lines via:

```bash
make qa-coverage
open target/llvm-cov/html/index.html   # if you also run `cargo llvm-cov --html`
```

### 3.10 `mutation` escaped

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

### 3.11 `semver` flagged

**Cause**: `cargo-semver-checks` found a breaking change in a publishable crate's public API since the last tag.

**Fix**:
- If unintentional: revert the breaking change.
- If intentional: it needs a 2.0. Inside 1.x there is no version bump that makes it acceptable — that is what the 1.0 guarantee says (`docs/api-rust.md`).

Soft during 0.x, hard from 1.0 on. It runs as a step of the `breaking-changes` gate below, which passes `--release-type minor` so the lints run whatever the version numbers say.

### 3.12 `breaking-changes` failed

**Cause**: a surface dora 1.x freezes changed. The report names the surface and the item — a removed `dora` flag, a reordered postcard field, a YAML property that became required, a raised `requires-python`.

**Fix**, in the order worth trying:

1. **Unintentional** (the common case): revert that part. The report quotes the old and new form, so the diff to undo is usually one line.
2. **You added something, and the gate is complaining that a generated snapshot is stale.** Run `make qa-breaking-update` and commit the result. Additions are fine; the snapshot diff is how they get reviewed.
3. **The change is genuinely needed and genuinely breaking.** Keep the old surface working alongside the new one — a deprecated alias, an added variant rather than a changed one, a new optional field rather than a required one. Removing the old form waits for 2.0.

Two failures that read oddly:

- **"major version bump ..."** — a 2.0 withdraws the promises the gate measures against, so it stops there rather than reporting a green or a red that means nothing. When the bump is deliberate, `ALLOW_MAJOR_BUMP=1 make qa-breaking` (and set the same variable on the `breaking-changes` job for the PR that carries it).
- **"the generated surface files are stale"** — regenerating changed the tree, so the comparison ran against an out-of-date snapshot and its "ok" meant nothing. Commit the regenerated files and read the report again.

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
# No miri-runnable target at present. The former one — dora-core's
# `metadata::tests`, exercising the unsafe pointer arithmetic in
# `ArrowTypeInfoExt::from_array` — was removed when the `ArrowTypeInfo`
# sidecar was dropped for Arrow-IPC framing; dora-core now has zero unsafe.
# The miri-worthy unsafe moved to dora-node-api's IPC encode/decode paths
# (`arrow_utils/ipc_encode.rs`, `event_stream`), but that crate links zenoh +
# shared-memory-server (`shm_open`), which miri cannot run wholesale. Add a
# tightly-scoped, FFI-free filter here once it has been verified to run clean:
# cargo +nightly miri test -p dora-node-api <ffi-free-ipc-test-filter>
```

**Do NOT** run miri on `shared-memory-server` — its tests call libc's `shm_open` which miri does not support. Every test aborts with "unsupported operation". See `plan-agentic-qa-strategy.md` Section T2.3 for the explanation and the long-term fix (Zenoh SHM migration).

---

## 6. Running property tests

Proptest strategies live inside `#[cfg(test)] mod tests` in the target source files. They run automatically under `cargo test`.

```bash
cargo test -p dora-message ws_protocol::tests::prop_
```

To increase the case count for a focused hunt:

```bash
PROPTEST_CASES=20000 cargo test -p dora-message ws_protocol::tests::prop_
```

Failed proptest cases are saved in `libraries/message/proptest-regressions/` and **should be committed** to source control — they re-run as fast regression checks on every test run.

---

## 7. Running focused mutation tests (when investigating a file)

```bash
# Mutation test a single file
# (Timeout 45s mirrors scripts/qa/mutants.sh default. Was 120s until the
# qa-nightly split; real passing tests fit comfortably under 45s and the
# shorter cap prevents broken-channel mutations from burning 2m each.)
cargo mutants --package <pkg> --file <path/to/file.rs> --jobs 4 --timeout 45

# List mutations without running (fast)
cargo mutants --package <pkg> --file <path/to/file.rs> --list

# Only mutations in the current diff (fast, for PR checks)
cargo mutants --in-diff origin/main --package <pkg>
```

**Tip**: mutation runs are expensive. When investigating a specific bug hypothesis, scope to a single file.

**Tip**: the full critical-crate chain takes about 17 hours (`dora-core`, `dora-message`, `dora-coordinator-store`, `dora-coordinator`, `dora-daemon`) with workspace-scoped tests. Budget accordingly.

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
