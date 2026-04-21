# Contributor QA Cheat Sheet

Short version: run the smallest validation that proves your change is correct, then include that evidence in the PR.

See also:

- [Contributing Guide](../CONTRIBUTING.md)
- [QA Runbook](qa-runbook.md)
- [Agentic QA Policy](agentic-qa-policy.md)

## 1. One-time setup

Run from Ubuntu or another Linux/macOS development machine:

```bash
sudo apt update
sudo apt install ripgrep

cargo install typos-cli cargo-audit cargo-deny cargo-llvm-cov cargo-mutants cargo-semver-checks
rustup component add llvm-tools-preview
```

If Cargo-installed tools are not found:

```bash
echo 'export PATH="$HOME/.cargo/bin:$PATH"' >> ~/.bashrc
source ~/.bashrc
```

The repo also provides:

```bash
make qa-install
```

`make qa-install` installs most Rust QA tools, but you still need `ripgrep` from your system package manager and `typos-cli` via `cargo install typos-cli`.

## 2. Daily workflow

Run everything from the repo root:

```bash
cd /path/to/dora
```

Typical loop:

1. For non-trivial work, open or discuss the change in a GitHub issue, discussion, or Discord thread first.
2. Add or update a failing test first.
3. Implement the smallest fix or feature that makes the test pass.
4. Run targeted validation while iterating.
5. Run the appropriate QA gate before opening or updating the PR.
6. In the PR description, include the validation you ran.

## 3. Fast command map

### Before commit

```bash
make qa-fast
```

Runs:

- `cargo fmt --all -- --check`
- `cargo clippy --all -- -D warnings` with Python crates excluded
- supply-chain audit
- unwrap-budget check
- typo check

### Before push

```bash
make qa-full
```

Runs `qa-fast` plus:

- full workspace tests
- coverage report
- optional adversarial review if supported tools are installed

### Target Tier 1 gate (stronger than today's CI)

```bash
make qa-deep    # alias: make qa-tier1
```

Runs `qa-full` plus the Tier 1 extras that stay laptop-only because
they're too slow for every PR (see `docs/plan-agentic-qa-strategy.md` §5):

- coverage (already in `qa-full`)
- adversarial LLM review (already in `qa-full`; skipped if tools missing)
- mutation testing (diff-scoped)
- semver checks

### Overnight run on a powerful machine (full CI nightly parity)

```bash
make qa-nightly    # ~100-120 min
```

Runs `qa-deep` plus property tests at 1000 cases, miri on unsafe code
(skipped if `cargo +nightly miri` isn't installed), **plus the
example-smoke suite** (`cargo test -p dora-examples --test example-smoke
-- --test-threads=1`), **plus `scripts/qa/ci-nightly-jobs.sh`**, which
drives the 6 CI-only GHA jobs locally (cluster-smoke, topic-and-top,
cpu-affinity, redb-backend, daemon-reconnect, state-reconstruction).

Together these cover all 11 GHA nightly test jobs. A green local run
predicts a green CI nightly schedule.

**Requires both `uv` and Python 3.12.** Before running example-smoke,
`scripts/qa/all.sh` preflights both prerequisites, then creates a
scratch venv at `target/qa-nightly-venv`, installs `pyarrow` and
`-e apis/python/node` into it, and runs `cargo test` with
`VIRTUAL_ENV` + `PATH` pointing at that venv. Mirrors the GHA
smoke-suite / log-sinks / service-action / streaming setup exactly
(GHA also provisions 3.12 explicitly via `actions/setup-python` before
the venv step) -- otherwise `dora run --uv` either falls through to
the system Python (ImportError on clean machines) or pulls PyPI
`dora-rs` which doesn't match the workspace message format (#1710).
If either prerequisite is missing, example-smoke fails fast with a
specific install hint:

```bash
# install uv
curl -LsSf https://astral.sh/uv/install.sh | sh

# install Python 3.12 (managed by uv; no apt/brew needed)
uv python install 3.12
```

The ci-nightly-jobs script installs the dora CLI into a scratch dir
(so it won't clobber your `~/.cargo/bin/dora`) and bails out if port
6013 is already in use. cpu-affinity-smoke and daemon-reconnect-smoke
skip on non-Linux because they rely on `sched_getaffinity` /
SIGSTOP+SIGCONT semantics.

**Does not** include full-repo mutation testing -- that's split into
`qa-mutation-audit` because it takes 10-18 hours on this workspace.

### Before tagging a release

```bash
make qa-release-gate
```

The automatable subset of Tier 3. The non-automatable parts (external
security audit, dogfood campaign, migration validation) are documented
in `docs/plan-agentic-qa-strategy.md` §7.

### Deliberate full-repo test-quality audit

```bash
make qa-mutation-audit    # ~10-18 hours
```

Runs `cargo-mutants --full` on the 6 critical crates (dora-core,
dora-daemon, dora-coordinator, dora-message, dora-coordinator-store,
shared-memory-server). About 1679 mutants. Background it, check the
morning after. Use when:

- Investigating low test coverage in a specific crate.
- Before a major release to score test-suite quality overall.
- Adding a new critical crate and want a baseline.

### Run all smoke-eligible example dataflows end-to-end

```bash
make qa-examples                       # full run, ~15-20 min
make qa-examples ARGS="--rust-only"    # Rust examples only
make qa-examples ARGS="-v"             # stream dora output live (debugging)
```

Wraps `scripts/smoke-all.sh`. The script deliberately skips examples
that need CUDA, ROS2, webcam, multi-machine deploy, C/C++ toolchains,
or interactive CLI -- run `scripts/smoke-all.sh -h` for the SKIP list.

Orthogonal to the qa-fast/full/deep ladder: those explicitly
`--exclude dora-examples` to keep per-commit and pre-push budgets
tight. Run this when you've touched node/operator APIs, CLI
subcommands, the descriptor (YAML) surface, or the
coordinator/daemon/runtime hot path -- anywhere an example dataflow
could silently break without failing any unit/integration test.

## 4. Most useful manual commands

### Workspace-level

```bash
cargo fmt --all -- --check

cargo clippy --all \
  --exclude dora-node-api-python \
  --exclude dora-operator-api-python \
  --exclude dora-ros2-bridge-python \
  -- -D warnings

cargo test --all \
  --exclude dora-node-api-python \
  --exclude dora-operator-api-python \
  --exclude dora-ros2-bridge-python \
  --exclude dora-cli-api-python \
  --exclude dora-examples
```

### While iterating on one crate

```bash
cargo test -p <crate>
cargo test -p <crate> <test_name>
cargo clippy -p <crate> -- -D warnings
```

## 5. Smoke, E2E, and contract tests

Use these when the change touches CLI, coordinator, daemon, dataflow parsing, or example behavior.

```bash
./scripts/smoke-all.sh
./scripts/smoke-all.sh --rust-only
./scripts/smoke-all.sh --python-only

cargo test --test example-smoke -- --test-threads=1
cargo test --test ws-cli-e2e -- --test-threads=1
cargo test --test fault-tolerance-e2e -- --test-threads=1
cargo test -p dora-examples --test example-smoke contract_ -- --test-threads=1
```

## 6. How much QA to run

Match the validation to the change class:

- Class A, low-risk docs or mechanical changes: `make qa-fast`
- Class B, behavior changes: `make qa-fast` plus targeted crate tests and relevant smoke or contract tests
- Class C, high-risk subsystem changes: Class B coverage plus full workspace tests, `fault-tolerance-e2e`, and contract tests

When in doubt, use the stricter class. The policy is in [agentic-qa-policy.md](agentic-qa-policy.md).

## 7. PR checklist

Before opening or updating a PR:

1. Rebase or merge as needed and confirm the branch still builds.
2. Run the QA level appropriate for the change.
3. Collect the exact commands and outcomes you ran.
4. Add a validation block to the PR description for Class B and C changes.

Minimal PR validation template:

```markdown
## Validation

- `make qa-fast`: ✅
- `cargo test -p <crate>`: ✅
- Relevant smoke / E2E / contract tests: ✅
```

## 8. Common setup failures

- `rg` missing: `sudo apt install ripgrep`
- `typos` missing: `cargo install typos-cli`
- `cargo-mutants` missing: `cargo install cargo-mutants`
- `cargo-semver-checks` missing: `cargo install cargo-semver-checks`
- Cargo tools installed but command not found: add `~/.cargo/bin` to `PATH`
