# Adora QA targets
#
# Local-first quality gates. Same scripts run in CI.
# See docs/plan-agentic-qa-strategy.md for the full strategy.

.PHONY: qa qa-fast qa-full qa-tier1 \
        qa-fmt qa-audit qa-unwrap qa-clippy qa-test qa-coverage qa-mutants qa-semver \
        qa-install

qa: qa-fast

qa-fast:
	@scripts/qa/all.sh --fast

qa-full:
	@scripts/qa/all.sh --full

qa-tier1:
	@scripts/qa/all.sh --tier1

# Individual gates

qa-fmt:
	@cargo fmt --all -- --check

qa-audit:
	@scripts/qa/audit.sh

qa-unwrap:
	@scripts/qa/unwrap-budget.sh

qa-clippy:
	@cargo clippy --all \
		--exclude adora-node-api-python \
		--exclude adora-operator-api-python \
		--exclude adora-ros2-bridge-python \
		-- -D warnings

qa-test:
	@cargo test --all \
		--exclude adora-node-api-python \
		--exclude adora-operator-api-python \
		--exclude adora-ros2-bridge-python \
		--exclude adora-cli-api-python \
		--exclude adora-examples

qa-coverage:
	@scripts/qa/coverage.sh

qa-mutants:
	@scripts/qa/mutants.sh

qa-semver:
	@scripts/qa/semver.sh

# One-shot tool installation

qa-install:
	cargo install cargo-audit cargo-deny cargo-llvm-cov cargo-mutants cargo-semver-checks
	rustup component add llvm-tools-preview
