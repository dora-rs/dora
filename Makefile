# Dora QA targets
#
# Local-first quality gates. Same scripts run in CI.
# See docs/plan-agentic-qa-strategy.md for the full strategy.
#
# The ladder (increasing thoroughness, increasing runtime):
#
#   make qa-fast             ~1 min      pre-commit sanity
#   make qa-full             ~5-10 min   pre-push
#   make qa-deep             ~15 min     target Tier 1 gate, stronger than today's CI
#                                        (adds coverage, adversarial, mutants, semver -
#                                        kept laptop-only, see strategy doc §5)
#   make qa-nightly         ~3-4 hours  Full parity with .github/workflows/nightly.yml
#                                        (qa-deep + proptest@1000 + miri + example-smoke
#                                        + ci-nightly-jobs). example-smoke covers the 5
#                                        example-backed GHA jobs; scripts/qa/ci-nightly-jobs.sh
#                                        drives the 6 remaining (cluster/topic-and-top/
#                                        cpu-affinity/redb-backend/daemon-reconnect/
#                                        state-reconstruction). Green local run predicts
#                                        a green CI nightly schedule.
#   make qa-release-gate                 Tier 3 automatable parts (deep + semver;
#                                        audit/dogfood are human)
#   make qa-mutation-audit   ~10-18 hrs  full-repo cargo-mutants; deliberate
#                                        test-quality audit, NOT every nightly
#
# Orthogonal to the ladder:
#
#   make qa-examples         ~15-20 min  run all smoke-eligible example
#                                        dataflows end-to-end (wraps
#                                        scripts/smoke-all.sh). The script
#                                        skips examples that need CUDA, ROS2,
#                                        webcam, multi-machine deploy, C/C++
#                                        toolchains, or interactive CLI --
#                                        run `scripts/smoke-all.sh -h` to see
#                                        the SKIP list. Not part of qa-*
#                                        ladder by design: the ladder
#                                        excludes dora-examples tests to
#                                        keep per-commit budgets tight.
#
# `make qa-tier1` is a back-compat alias for `make qa-deep`.

.PHONY: qa qa-fast qa-full qa-deep qa-tier1 qa-nightly qa-release-gate qa-mutation-audit \
        qa-examples \
        qa-fmt qa-audit qa-unwrap qa-clippy qa-test qa-coverage qa-mutants qa-semver \
        qa-adversarial qa-install

qa: qa-fast

qa-fast:
	@scripts/qa/all.sh --fast

qa-full:
	@scripts/qa/all.sh --full

qa-deep:
	@scripts/qa/all.sh --deep

# Back-compat alias. Prefer `make qa-deep` in new docs/scripts.
qa-tier1: qa-deep

qa-nightly:
	@scripts/qa/all.sh --nightly

qa-release-gate:
	@scripts/qa/all.sh --release-gate

qa-mutation-audit:
	@scripts/qa/all.sh --mutation-audit

# Run all smoke-eligible example dataflows end-to-end via
# scripts/smoke-all.sh. Skips examples requiring CUDA, ROS2, webcam,
# multi-machine deploy, C/C++ toolchains, or interactive CLI.
# Budget ~15-20 min. Pass flags through, e.g.
#   make qa-examples ARGS="--rust-only"
#   make qa-examples ARGS="-v"    # stream dora output live
qa-examples:
	@scripts/smoke-all.sh $(ARGS)

# Individual gates

qa-fmt:
	@cargo fmt --all -- --check

qa-audit:
	@scripts/qa/audit.sh

qa-unwrap:
	@scripts/qa/unwrap-budget.sh

qa-clippy:
	@cargo clippy --all \
		--exclude dora-node-api-python \
		--exclude dora-operator-api-python \
		--exclude dora-ros2-bridge-python \
		-- -D warnings

qa-test:
	@cargo test --all \
		--exclude dora-node-api-python \
		--exclude dora-operator-api-python \
		--exclude dora-ros2-bridge-python \
		--exclude dora-cli-api-python \
		--exclude dora-examples

qa-coverage:
	@scripts/qa/coverage.sh

qa-mutants:
	@scripts/qa/mutants.sh

qa-semver:
	@scripts/qa/semver.sh

# Adversarial LLM review of current diff (requires codex or claude CLI)
qa-adversarial:
	@scripts/qa/adversarial.sh

# One-shot tool installation

qa-install:
	cargo install cargo-audit cargo-deny cargo-llvm-cov cargo-mutants cargo-semver-checks
	rustup component add llvm-tools-preview
