#!/usr/bin/env bash
# scripts/qa/all.sh — master QA runner
#
# Runs the QA gates from docs/plan-agentic-qa-strategy.md.
# Designed for local-first execution: same script runs locally and in CI.
#
# Modes (increasing thoroughness):
#   --fast            ~1 min     pre-commit sanity (fmt, clippy, audit, unwrap, typos)
#   --full            ~5-10 min  pre-push (fast + tests + coverage + optional adversarial)
#   --deep            ~15 min    target Tier 1 gate, stronger than today's CI
#                                (full + mutants on diff + semver; see strategy doc §5 for why the
#                                extras are laptop-only)
#   --tier1                      back-compat alias for --deep
#   --nightly         ~3-4 hours   Full parity with .github/workflows/nightly.yml
#                                (deep + proptest@1000 + miri + example-smoke +
#                                ci-nightly-jobs). After the #1716 rebalance,
#                                nightly.yml has 18 test jobs: example-smoke
#                                covers 4 (smoke-suite, log-sinks, service-action,
#                                streaming); ci-nightly-jobs.sh drives the 14
#                                remaining with platform-aware dispatch
#                                (record-replay, cluster-smoke, topic-and-top-smoke,
#                                cpu-affinity-smoke [Linux], redb-backend-smoke,
#                                daemon-reconnect-smoke [Linux],
#                                state-reconstruction-smoke, test-cross-platform,
#                                examples, cli-tests, bench-example, cross-check,
#                                ros2-bridge [Linux+ROS2], msrv). Green local
#                                qa-nightly on platform X predicts a green CI
#                                nightly for platform X's jobs.
#                                Requires BOTH `uv` AND Python 3.12 (preflighted
#                                with specific install hints; matches GHA's
#                                actions/setup-python@3.12 + uv setup).
#   --release-gate               Tier 3 automatable (deep + semver; audit+dogfood are human gates)
#   --mutation-audit  ~10-18 hrs full-repo cargo-mutants across 6 critical crates
#                                (1679+ mutants). Deliberate test-quality audit, not every nightly.
#
# Without flags, runs --fast.

set -euo pipefail

cd "$(dirname "$0")/../.."

MODE="${1:---fast}"
FAILED=()

run() {
  local name="$1"; shift
  echo
  echo "=== $name ==="
  if "$@"; then
    echo "  PASS: $name"
  else
    echo "  FAIL: $name"
    FAILED+=("$name")
  fi
}

# Optional run: skip (without failing) if the required tool is missing.
# Used for Tier 2 items like miri that need nightly Rust.
run_optional() {
  local name="$1" check="$2"; shift 2
  if ! eval "$check" > /dev/null 2>&1; then
    echo
    echo "=== $name (SKIP: prerequisite missing) ==="
    return
  fi
  run "$name" "$@"
}

# Normalize --tier1 to --deep so the rest of the script only branches on one.
if [[ "$MODE" == "--tier1" ]]; then
  echo "(note: --tier1 is a deprecated alias for --deep)"
  MODE="--deep"
fi

# Validate mode up front. An unknown mode should fail fast with zero side
# effects -- do NOT run fast gates then error out at the end.
case "$MODE" in
  --fast|--full|--deep|--nightly|--release-gate|--mutation-audit)
    ;;
  *)
    echo "Unknown mode: $MODE"
    echo "Usage: $0 [--fast|--full|--deep|--tier1|--nightly|--release-gate|--mutation-audit]"
    exit 2
    ;;
esac

# Print a per-mode overview so a developer knows what they're about to
# sit through. The actual checks follow the same order listed here.
print_overview() {
  local mode="$1"
  local header
  case "$mode" in
    --fast)
      header="qa-fast -- pre-commit sanity (~1 min)"
      cat <<EOF
============================================================
$header
Will run:
  1. fmt            -- cargo fmt --all -- --check
  2. clippy         -- cargo clippy --all -- -D warnings (excluding Python)
  3. audit          -- cargo-audit + cargo-deny on the dependency tree
  4. unwrap-budget  -- count production .unwrap() / .expect( regressions
  5. typos          -- spell-check against _typos.toml allowlist
============================================================
EOF
      ;;
    --full)
      header="qa-full -- pre-push gate (~5-10 min)"
      cat <<EOF
============================================================
$header
Will run:
  1-5. everything from qa-fast                  (fmt/clippy/audit/unwrap/typos)
  6.   test         -- cargo test --all         (workspace test suite)
  7.   coverage     -- cargo llvm-cov           (writes lcov.info)
  8.   adversarial  -- codex/claude review      (optional; skipped if unavailable)
============================================================
EOF
      ;;
    --deep)
      header="qa-deep -- target Tier 1 gate, stronger than today's CI (~15 min)"
      cat <<EOF
============================================================
$header
Today's CI PR gate only runs a subset of this: fmt, clippy, typos,
audit, unwrap-budget, and the workspace test suite. qa-deep adds the
planned Tier 1 extras (see docs/plan-agentic-qa-strategy.md §5) that
are kept laptop-only today because they're too slow for every PR:
coverage, adversarial review, diff-scoped mutation testing, semver.

Will run:
  1-5.  everything from qa-fast                 (in CI today)
  6.    test         -- cargo test --all        (in CI today)
  7.    coverage     -- cargo llvm-cov          (NOT in CI; laptop-only)
  8.    adversarial  -- codex/claude review     (NOT in CI; skipped w/o tools)
  9.    mutants      -- cargo-mutants on diff   (NOT in CI; laptop-only)
  10.   semver       -- cargo-semver-checks     (NOT in CI; pre-release only)
============================================================
EOF
      ;;
    --nightly)
      header="qa-nightly -- Full parity with .github/workflows/nightly.yml (~3-4 hours)"
      cat <<EOF
============================================================
$header
For overnight runs on a powerful machine. Will run:
  1-5.  everything from qa-fast
  6-8.  everything from qa-full                 (test, coverage, adversarial)
  9.    mutants (diff-scoped)                   -- same as qa-deep
  10.   semver                                  -- cargo-semver-checks vs last tag
  11.   proptest @ 1000 cases per property      (vs 50 cases in Tier 1)
  12.   miri                                    -- undefined-behavior check (SKIP if cargo +nightly miri missing)
  13.   example-smoke                           -- tests/example-smoke.rs (52 tests;
                                                   covers GHA smoke-suite + log-sinks
                                                   + service-action + streaming).
                                                   Runs inside a scratch uv venv that
                                                   has \`-e apis/python/node\` installed,
                                                   matching the GHA Python setup exactly
                                                   (so workspace Python bindings are used,
                                                   NOT PyPI). Requires uv.
  14.   ci-nightly-jobs                         -- scripts/qa/ci-nightly-jobs.sh
                                                   Platform-aware: runs the subset of GHA
                                                   nightly jobs that applies to the dev's OS.
                                                   Covers record-replay, cluster-smoke,
                                                   topic-and-top, cpu-affinity [Linux],
                                                   redb-backend, daemon-reconnect [Linux],
                                                   state-reconstruction, test-cross-platform,
                                                   examples, cli-tests, bench-example, msrv,
                                                   cross-check, ros2-bridge [Linux+ROS2].

Steps 13 + 14 together cover all 18 GHA nightly test jobs. A green
local qa-nightly on platform X predicts a green CI nightly schedule
for platform X's jobs. (Cross-platform jobs that the dev's OS can't
run -- e.g. ros2-bridge on macOS -- SKIP locally with a clear note.)

Why record-replay lives in ci-nightly-jobs, not example-smoke:
example-smoke's contract_record_replay_* uses a fixture that strips
`build:` directives to sidestep the /tmp cargo bug class. That means
it exercises the data path but not the cargo-build-during-replay
path -- which has been a recurring regression site (#1673/#1674,
#1691). The ci-nightly-jobs record-replay function runs against the
stock examples/rust-dataflow/dataflow.yml with build directives
INTACT, catching that class of regression locally.

Platform-specific SKIP behavior:
  - cpu-affinity-smoke, daemon-reconnect-smoke: Linux-only
    (sched_getaffinity / SIGSTOP semantics).
  - test-cross-platform: skipped on Linux (redundant with qa-test);
    runs on macOS/Windows.
  - examples: full on Linux, all-minus-cmake on macOS, Rust-only on Windows.
  - cli-tests C/C++ template tests: Linux-only per GHA.
  - cross-check: native target for dev's OS only; full cross matrix is CI.
  - ros2-bridge: Linux + /opt/ros/humble/setup.bash required.

Full-repo mutation testing is a SEPARATE target (qa-mutation-audit)
because in practice it takes 10-18 hours on this workspace -- too long
to bundle with a "nightly" run most devs will actually use. See
docs/plan-agentic-qa-strategy.md §6.
============================================================
EOF
      ;;
    --mutation-audit)
      header="qa-mutation-audit -- full-repo cargo-mutants (10-18 hours)"
      cat <<EOF
============================================================
$header
Runs cargo-mutants --full on 6 critical crates: dora-core, dora-daemon,
dora-coordinator, dora-message, dora-coordinator-store,
shared-memory-server. About 1679 mutants last measured.

A test-quality audit, NOT a code gate. Run when:
  - Investigating low test coverage in a specific crate.
  - Before a major release to score test-suite quality overall.
  - Adding a new critical crate and want a baseline.

Expect 10-18 hours on a fast machine. Background it and check the
morning after. Timeouts are capped at 45s per mutant (was 120s,
lowered after observed 52 full-timeout waits per 1008 mutants).
============================================================
EOF
      ;;
    --release-gate)
      header="qa-release-gate -- Tier 3 automatable subset"
      cat <<EOF
============================================================
$header
The automatable parts of the Tier 3 release gate. Will run:
  1-5.   everything from qa-fast
  6-8.   everything from qa-full                 (test, coverage, adversarial)
  9.     mutants (diff-scoped)
  10.    semver
Non-automatable Tier 3 gates (external security audit, 7-day dogfood
campaign, migration validation on external repos) are human-gated --
see docs/plan-agentic-qa-strategy.md §7.
============================================================
EOF
      ;;
  esac
}

print_overview "$MODE"

# ----- Always run (fast) -----
run "fmt"           cargo fmt --all -- --check
run "clippy"        cargo clippy --all \
                      --exclude dora-node-api-python \
                      --exclude dora-operator-api-python \
                      --exclude dora-ros2-bridge-python \
                      -- -D warnings
run "audit"         scripts/qa/audit.sh
run "unwrap-budget" scripts/qa/unwrap-budget.sh
run "typos"         scripts/qa/typos.sh

case "$MODE" in
  --fast)
    : # done
    ;;
  --full|--deep|--nightly|--release-gate)
    run "test" cargo test --all \
      --exclude dora-node-api-python \
      --exclude dora-operator-api-python \
      --exclude dora-ros2-bridge-python \
      --exclude dora-cli-api-python \
      --exclude dora-examples
    run "coverage" scripts/qa/coverage.sh
    # Adversarial LLM review skips cleanly on machines without codex/claude.
    run "adversarial" scripts/qa/adversarial.sh --optional
    ;;
  --mutation-audit)
    # Skips the common test/coverage/adversarial path -- this mode is
    # explicitly about mutation testing, nothing else.
    :
    ;;
esac

case "$MODE" in
  --deep|--nightly|--release-gate)
    run "mutants (diff-scoped)" scripts/qa/mutants.sh
    run "semver"                scripts/qa/semver.sh
    ;;
esac

case "$MODE" in
  --nightly)
    # Tier 2 locally. See docs/plan-agentic-qa-strategy.md §6.
    #
    # proptest: the existing property tests run under `cargo test` with a
    # reduced case count in Tier 1. Here we bump to 1000 cases per property.
    run "proptest-1000" env PROPTEST_CASES=1000 cargo test --all \
      --exclude dora-node-api-python \
      --exclude dora-operator-api-python \
      --exclude dora-ros2-bridge-python \
      --exclude dora-cli-api-python \
      --exclude dora-examples \
      -- proptest

    # miri requires nightly Rust + miri component. Skip (with note) if missing.
    #
    # Scoped to the `metadata::tests::` module only. Those tests were
    # specifically written to exercise the unsafe pointer-arithmetic
    # path in `ArrowTypeInfoExt::from_array` under miri (see the
    # module doc comment and docs/plan-agentic-qa-strategy.md §T2.3).
    # Running `cargo miri test -p dora-core` without a filter tries
    # every test in the crate; most fail because they use `tempfile`
    # or other filesystem ops that miri's isolation sandbox rejects.
    run_optional "miri" "cargo +nightly miri --version" \
      cargo +nightly miri test -p dora-core -- metadata::tests

    # Ambient Python venv for example-smoke. CI smoke jobs all set one up
    # with `uv pip install -e apis/python/node` before running cargo test;
    # without it, `dora run --uv` creates isolated venvs that either fall
    # through to system Python for `dora` bindings (ImportError on clean
    # machines) or pull dora-rs from PyPI (#1710: PyPI 0.5.0 speaks message
    # format v0.8.0, workspace speaks v0.2.1 -> mismatch).
    #
    # Without this step, `make qa-nightly` passes locally only when the
    # user already has workspace bindings in some other ambient venv -- a
    # hidden prerequisite that defeats the whole point of CI parity.
    # Hard prereqs for this step: uv + an available Python 3.12 interpreter
    # (matches .github/workflows/nightly.yml:56 which installs 3.12 via
    # actions/setup-python before creating the venv). Checking both up
    # front -- a missing 3.12 would otherwise fail at `uv venv -p 3.12`
    # below with a raw uv error instead of our clear install hint.
    nightly_py_missing() {
      local name="$1"
      echo
      echo "=== example-smoke (FAIL: $name prerequisite missing) ==="
      echo "Install with:"
      if [ "$name" = "uv" ]; then
        echo "  curl -LsSf https://astral.sh/uv/install.sh | sh"
      else
        echo "  uv python install 3.12"
      fi
      echo "Alternative: skip Python smoke tests with \`make qa-deep\`"
      echo "(Tier 1 gate; no Python smoke, no uv/3.12 requirement)."
      FAILED+=("example-smoke: $name missing")
    }

    if ! command -v uv > /dev/null 2>&1; then
      nightly_py_missing "uv"
    elif ! uv python find 3.12 > /dev/null 2>&1; then
      nightly_py_missing "Python 3.12"
    else
      NIGHTLY_VENV="$(pwd)/target/qa-nightly-venv"
      echo
      echo "=== preparing ambient Python venv at $NIGHTLY_VENV ==="
      echo "=== (matches GHA smoke-suite's 'uv pip install -e apis/python/node') ==="
      # Fresh venv each run so `-e apis/python/node` picks up workspace
      # bindings rebuilt by maturin since the last run.
      rm -rf "$NIGHTLY_VENV"
      uv venv --seed -p 3.12 "$NIGHTLY_VENV" > /dev/null
      VIRTUAL_ENV="$NIGHTLY_VENV" uv pip install --quiet pyarrow
      VIRTUAL_ENV="$NIGHTLY_VENV" uv pip install --quiet -e apis/python/node

      # CI-nightly parity: the GHA `smoke-suite` job runs exactly this
      # command, and the log-sinks / service-action / streaming /
      # record-replay jobs are all backed by tests in the same file.
      # --test-threads=1 matches CI (the smoke tests share global
      # coordinator ports and can't run in parallel).
      run "example-smoke" env \
        VIRTUAL_ENV="$NIGHTLY_VENV" \
        PATH="$NIGHTLY_VENV/bin:$PATH" \
        cargo test -p dora-examples --test example-smoke \
        -- --test-threads=1
    fi

    # Drive the 14 remaining GHA nightly jobs with platform-aware dispatch
    # (record-replay, cluster-smoke, topic-and-top, cpu-affinity [Linux],
    # redb-backend, daemon-reconnect [Linux], state-reconstruction,
    # test-cross-platform, examples, cli-tests, bench-example, cross-check,
    # ros2-bridge [Linux+ROS2], msrv). Jobs that can't run on the dev's OS
    # SKIP cleanly. The script installs dora CLI into a scratch dir, so it
    # won't clobber the user's ~/.cargo/bin/dora. See #1707 + #1716 for the
    # alignment rationale.
    run "ci-nightly-jobs" scripts/qa/ci-nightly-jobs.sh
    ;;
  --mutation-audit)
    # Deliberate full-repo mutation audit. Expect 10-18 hours.
    run "mutants (full-repo)" scripts/qa/mutants.sh --full
    ;;
esac

echo
echo "============================================================"
if [[ ${#FAILED[@]} -eq 0 ]]; then
  echo "All checks passed."
  exit 0
else
  echo "FAILED:"
  printf '  - %s\n' "${FAILED[@]}"
  exit 1
fi
