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
| Service / Action / Streaming communication patterns | `service-action-streaming` |
| Record / replay round-trip | `record-replay` |

Run locally:
```bash
cargo test -p dora-examples --test example-smoke -- --test-threads=1
dora run examples/service-example/dataflow.yml --stop-after 15s
dora run examples/action-example/dataflow.yml --stop-after 20s
dora run examples/log-sink-file/dataflow.yml --stop-after 15s
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

### Cluster mode

```bash
dora cluster up --size 3
dora cluster status
dora start examples/multiple-daemons/dataflow.yml
dora cluster down
```

Not automated: cluster up spawns multiple daemon processes and requires
network orchestration that isn't stable in GitHub Actions runners.

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

- `dora topic echo/hz/info/pub` TUI interaction
- `dora top` (TUI)
- `dora graph` output validation (generates Mermaid; no golden file compare)
- `dora param get/set/delete` interactive path (covered by `ws-cli-e2e` on the wire)
- `dora trace list/view`
- `dora self update`

Most of these are interactive TUI commands that need an expect-style harness.
Opening separate issues as we pick them up.

## Promotion policy

If a nightly failure reproduces for **3 consecutive runs**, promote the
corresponding test to the remote CI gate (move from `nightly.yml` to
`ci.yml`). Keep lean-CI-first: only promote gates that catch real bugs,
not flaky infra.

If a remote CI gate is flaky (>10% false-positive rate over a month),
demote it to nightly.
