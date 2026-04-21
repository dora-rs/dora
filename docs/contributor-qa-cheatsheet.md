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

### Overnight run on a powerful machine (Tier 2 equivalent)

```bash
make qa-nightly    # ~4 hours
```

Runs `qa-deep` plus property tests at 1000 cases, miri on unsafe code
(skipped if `cargo +nightly miri` isn't installed), and full-repo
mutation testing.

### Before tagging a release

```bash
make qa-release-gate
```

The automatable subset of Tier 3. The non-automatable parts (external
security audit, dogfood campaign, migration validation) are documented
in `docs/plan-agentic-qa-strategy.md` §7.

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
