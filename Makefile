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
#   make qa-nightly          ~30-60 min  Tier 2 locally (deep + proptest@1000 + miri)
#   make qa-release-gate                 Tier 3 automatable parts (deep + semver;
#                                        audit/dogfood are human)
#   make qa-mutation-audit   ~10-18 hrs  full-repo cargo-mutants; deliberate
#                                        test-quality audit, NOT every nightly
#
# Orthogonal to the ladder:
#
#   make qa-examples         ~15-20 min  run every example dataflow end-to-end
#                                        (wraps scripts/smoke-all.sh). Not part
#                                        of qa-* ladder by design -- the ladder
#                                        explicitly excludes dora-examples tests
#                                        to keep per-commit / pre-push budgets
#                                        tight. Run this when you want actual
#                                        dataflows exercised.
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

# Exercise every example dataflow end-to-end. Wraps scripts/smoke-all.sh.
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
