# Testing Matrix

Where each feature is tested, and how to run the manual gates.

See issue #215 for the original audit. See `CLAUDE.md` for the CI philosophy
(remote CI is deliberately lean; deep gates run on laptops and nightly).

## Tier 0 — Remote CI (every push and PR)

Fast gates. Must pass for every PR.

| Area | Where | How |
|---|---|---|
| Format | `.github/workflows/ci.yml` `fmt` | `cargo fmt --all -- --check` |
| Clippy | `.github/workflows/ci.yml` `clippy` | `cargo clippy --all -- -D warnings` |
| Core tests | `.github/workflows/ci.yml` `test` | `cargo test --all` (Python crates excluded) |
| CLI smoke (argparse, `validate`, `expand`, `graph`) | `.github/workflows/ci.yml` `test` | Per-subcommand `--help` + validate/expand/graph on representative YAMLs (3 platforms) |
| Example dataflows (Rust/C/C++/Python) | `.github/workflows/ci.yml` `examples` | `cargo run --example ...` on 3 platforms |
| CLI template + basic Python examples | `.github/workflows/ci.yml` `cli` | `dora new` + `dora run` on 3 platforms |
| E2E WS control plane | `.github/workflows/ci.yml` `e2e` | `cargo test --test ws-cli-e2e` |
| Fault tolerance E2E | `.github/workflows/ci.yml` `fault-tolerance-e2e` | `cargo test --test fault-tolerance-e2e` |
| Supply chain (cargo-audit / cargo-deny) | `.github/workflows/ci.yml` `audit` | `scripts/qa/audit.sh` |
| Unwrap budget | `.github/workflows/ci.yml` `unwrap-budget` | `scripts/qa/unwrap-budget.sh` |
| Benchmark regression | `.github/workflows/ci.yml` `benchmark-regression` | |
| Typos | `.github/workflows/ci.yml` `typos` | `crate-ci/typos` |

## Tier 1 — Nightly (`.github/workflows/nightly.yml`)

Runs daily at 06:00 UTC and via `workflow_dispatch`. Failures file issues with
the `nightly-regression` label but do not block PRs.

| Area | Job |
|---|---|
| Full smoke suite (`tests/example-smoke.rs`, 44 tests) | `smoke-suite` |
| Log-sink examples (file, alert, tcp) | `log-sinks` |
| Service / Action patterns (Rust-only) | `service-action` |
| Streaming example (Python nodes) | `streaming` |
| Record / replay round-trip | `record-replay` |
| Cluster lifecycle (`cluster status`, `cluster down`) | `cluster-smoke` |
| Inspection commands (`top --once`, `topic list/info/pub`) | `topic-and-top-smoke` |
| `cpu_affinity` end-to-end (mask actually applied) | `cpu-affinity-smoke` (Linux only) |
| redb coordinator store survives restart | `redb-backend-smoke` |
| Daemon auto-reconnect (SIGSTOP past heartbeat → SIGCONT → re-register) | `daemon-reconnect-smoke` (Linux only) |
| State reconstruction (partial: store hydrate + Running → Recovering) | `state-reconstruction-smoke` |

Run locally:
```bash
# Smoke suite
cargo test -p dora-examples --test example-smoke -- --test-threads=1

# Rust-only examples (no Python setup needed)
dora run examples/service-example/dataflow.yml --stop-after 15s
dora run examples/action-example/dataflow.yml --stop-after 20s

# Python-backed examples — need uv venv + local dora install first:
#   uv venv --seed -p 3.12
#   source .venv/bin/activate
#   uv pip install pyarrow
#   uv pip install -e apis/python/node
dora run examples/log-sink-file/dataflow.yml --uv --stop-after 15s
dora run examples/streaming-example/dataflow.yml --uv --stop-after 15s
```

## Tier 2 — Laptop / manual

Not automated — too heavy, platform-specific, or requires dedicated infra.
Run before releases or when touching the relevant subsystem.

### Deep QA gates

Documented in `CLAUDE.md`. Invoke via `make`:

| Gate | Command | When |
|---|---|---|
| Full QA (fast + tests + coverage) | `make qa-full` | before significant push |
| Tier 1 (qa-full + mutants + semver) | `make qa-tier1` | before release, or crate audit |
| Coverage (lcov report) | `make qa-coverage` | when investigating coverage |
| Mutation testing | `make qa-mutants` | when auditing test quality |
| Semver check | `make qa-semver` | before publishing to crates.io |
| Adversarial LLM review | `make qa-adversarial` | before merging AI-authored code (requires codex or claude CLI) |

### Cluster mode (SSH-backed `cluster up`)

```bash
# cluster.yml describes coordinator + remote machines
dora cluster up cluster.yml
dora cluster status
dora start examples/multiple-daemons/dataflow.yml
dora cluster down
```

Not automated: `dora cluster up` SSH-es into each machine in cluster.yml,
which requires real remote hosts or a configured localhost SSH setup
that's not available on stock GHA runners.

**What is automated** (Tier 1 nightly `cluster-smoke` job): `dora cluster
status` and `dora cluster down` against a local `dora up` coordinator.
Covers the coordinator-side wire protocol for those subcommands. The
SSH-dependent `cluster up` path needs dedicated infra.

### Soft real-time

```bash
sudo dora daemon --rt       # requires CAP_SYS_NICE or root
# With cpu_affinity + SCHED_FIFO in the dataflow YAML
```

Not automated: requires specific kernel config (`CONFIG_RT_GROUP_SCHED`)
and privileged execution, which GHA runners do not provide reliably.

### Coordinator HA / distributed

```bash
# Terminal 1
dora coordinator --backend redb --store-path /tmp/dora-coord
# Terminal 2
dora daemon --machine-id A --coordinator-addr 127.0.0.1
# Terminal 3 — kill coordinator, restart, verify daemon auto-reconnects
pkill dora-coordinator
dora coordinator --backend redb --store-path /tmp/dora-coord
```

Not automated: failure-injection timing is flaky on cloud runners.
Planned: dedicated self-hosted runner for HA scenarios.

### Cross-machine Zenoh data plane

```bash
# Machine A
dora coordinator --bind 0.0.0.0
dora daemon --machine-id A --coordinator-addr <machine-a-ip>
# Machine B
dora daemon --machine-id B --coordinator-addr <machine-a-ip>
```

Not automated: requires two networked hosts.

## What's NOT covered anywhere yet

Tracked in issue #215. Selected items:

- `dora self update` (destructive path — binary swap; `--check-only` is covered in nightly but the actual upgrade requires a sandbox harness)
- `dora top` interactive mode (non-`--once`)

Most of these are interactive TUI commands that need either non-interactive
modes or an expect-style harness. Opening separate issues as we pick them
up.

### Already covered (despite earlier audit)

These were on the gap list but now have nightly coverage in
`topic-and-top-smoke`:
- `dora top --once` (JSON snapshot)
- `dora topic list --format json` (NDJSON parse)
- `dora topic info --duration N` (asserts `Total messages >= 10` on 10Hz fixture — regression guard for #236)
- `dora topic echo --count N` (asserts N frames of correct topic name)
- `dora topic hz --duration N` (asserts `samples >= 10` on 10Hz fixture)
- `dora topic pub --count N`
- `dora trace list` (empty-state message)
- `dora trace view <absent-uuid>` (empty-spans message, exit 0)
- `dora trace view <bad-prefix>` (clear error message, exit non-zero)
- `dora self update --check-only` (read-only path against live GitHub releases; tolerates API rate limits)

## Platform parity

C/C++ template CI coverage is intentionally scoped to Linux (see issue #230):

| Test | Linux | macOS | Windows |
|---|---|---|---|
| `dora new --lang rust/python` scaffolding | Tier 0 (`cli` job) | Tier 0 (`cli` job) | Tier 0 (`cli` job) |
| `dora new --lang c/cxx` scaffolding + CMake build + `dora run` | Tier 0 (`cli` job) | not covered | not covered |
| `cxx-dataflow`, `c-dataflow` examples | Tier 0 (`examples` job) | Tier 0 | Tier 0 |
| `cxx-arrow-dataflow` (needs Arrow C++ lib) | Tier 0 | Tier 0 (homebrew `apache-arrow`) | not covered |
| `cmake-dataflow` example | Tier 0 | not covered | not covered |

**Rationale.** Production C/C++ node users are Linux-heavy. macOS/Windows C/C++ CI
would need vcpkg or homebrew-formula pinning per runner, and the cost/benefit is
poor for the number of real-world users on those paths. Rust/Python coverage stays
cross-platform because those are the primary audience. If a macOS or Windows C/C++
regression is reported, the fix is to lift the `if: runner.os == 'Linux'` gate on
the relevant step and add platform-specific install logic — not to keep shipping
broken.

## Promotion policy

If a nightly failure reproduces for **3 consecutive runs**, promote the
corresponding test to the remote CI gate (move from `nightly.yml` to
`ci.yml`). Keep lean-CI-first: only promote gates that catch real bugs,
not flaky infra.

If a remote CI gate is flaky (>10% false-positive rate over a month),
demote it to nightly.
