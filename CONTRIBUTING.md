# How to contribute to `dora-rs`

We welcome bug reports, feature requests, and pull requests!

Please discuss non-trivial changes in a Github issue or on Discord first before implementing them.
This way, we can avoid unnecessary work on both sides.

## Building

The `dora` project is set up as a [cargo workspace](https://doc.rust-lang.org/cargo/reference/workspaces.html).
You can use the standard `cargo check`, `cargo build`, `cargo run`, and `cargo test` commands.
To run a command for a specific package only, pass e.g. `--package dora-daemon`.
Running a command for the whole workspace is possible by passing `--workspace`.

```bash
# Build all (excluding Python packages which require maturin)
cargo build --all --exclude dora-node-api-python --exclude dora-operator-api-python --exclude dora-ros2-bridge-python

# Test all
cargo test --all --exclude dora-runtime-python --exclude dora-node-api-python --exclude dora-operator-api-python --exclude dora-ros2-bridge-python

# Lint
cargo clippy --all -- -D warnings

# Format
cargo fmt --all
```

## Development Workflow

1. **Plan**: Use an issue or discussion to outline your approach
2. **Test-Driven**: Write a failing test first (RED), then make it pass (GREEN), then refactor (IMPROVE)
3. **Review**: Run `cargo clippy` and `cargo fmt` before submitting
4. **Commit**: Use conventional commit format: `type(scope): description`
   - Types: `feat`, `fix`, `refactor`, `docs`, `test`, `chore`, `perf`, `ci`

See [docs/testing-guide.md](docs/testing-guide.md) for detailed testing patterns.

## Continuous Integration (CI)

We use [GitHub Actions](https://github.com/features/actions) to run automated checks on all commits and pull requests.
Please ensure that your pull request passes all checks. You don't need to fix warnings unrelated to your changes.

Current CI jobs:

- **fmt**: `cargo fmt --all -- --check`
- **clippy**: `cargo clippy --all -- -D warnings` (excluding Python packages)
- **test**: `cargo test --all` (excluding Python and example packages)
- **e2e**: End-to-end smoke tests, fault tolerance tests, WebSocket CLI tests
- **typos**: Spell checking via `crate-ci/typos`

## Issue Management

### Dora Bot

We use a custom Github Action to help manage issue assignments. You can interact with this action using the following:

- `@dora-bot assign me` - Assigns the current issue to you.
- `@dora-bot unassign me` - Removes yourself from the issue assignment.

For maintainers only:
- `dora-bot unassign @username` - Allows maintainers to unassign other contributors
Note: All issue assignments will be removed automatically after 2 weeks of inactivity.

## Style

We use [`rustfmt`](https://github.com/rust-lang/rustfmt) with its default settings to format our code.
Please run `cargo fmt --all` on your code before submitting a pull request.
Our CI will run an automatic formatting check of your code.

## Releasing

Releases are automated via GitHub Actions. Only maintainers should cut releases.

### Prerequisites (one-time)

```bash
cargo install cargo-release git-cliff
```

### Cutting a release

```bash
# 1. Ensure main is green and up to date
git checkout main && git pull

# 2. Generate/update changelog
git cliff --tag v0.5.0 --output CHANGELOG.md
git add CHANGELOG.md && git commit -m "docs: update changelog for v0.5.0"

# 3. Bump version, commit, tag, and push (dry-run first)
cargo release minor              # dry-run: review what will happen
cargo release minor --execute    # bumps workspace version, tags v0.5.0, pushes
```

If `main` is protected, `cargo release --execute` cannot push the bump commit to it. Land the version bump as a normal PR instead, then tag the merge commit by hand — `release.yml` verifies the tag against `[workspace.package].version` and fails fast on skew.

`release.yml` takes over from the tag push and does the whole release in one run:
- Publishes all crates to crates.io, in dependency order
- Builds and publishes **both** wheels to PyPI — `dora-rs` and `dora-rs-cli` — across the full platform matrix plus sdists, by calling `pip-release.yml`
- Builds the C/C++ libraries for 4 targets, by calling `publish-c-cpp-libraries.yml`
- Builds CLI binaries for all platforms
- Creates a GitHub Release with the changelog, the CLI binaries, the install scripts, and the C/C++ archives
- **Verifies the release is complete** and fails if it is not

That last step exists because it used to be possible to ship a green but half-finished release. The wheel and C/C++ jobs lived in workflows triggered only by `release: published`, and a release created by `GITHUB_TOKEN` raises no such event — so `v1.0.0-rc.4` published 4 of `dora-rs`'s 11 files, no `dora-rs-cli` at all, and none of the 8 C/C++ archives, with every job green. Those workflows are now *called* by `release.yml` on the tag push, and `verify-release` reads back what is actually on crates.io, PyPI and the GitHub Release, failing the run naming anything absent.

To audit an already-published tag:

```bash
make qa-verify-release VERSION=1.0.0-rc.5
```

### Configuration

| File | Purpose |
|------|---------|
| `release.toml` | cargo-release settings (version bump, tagging) |
| `cliff.toml` | git-cliff changelog generation |
| `.github/workflows/release.yml` | Tag-triggered publish pipeline; calls the two workflows below |
| `.github/workflows/pip-release.yml` | Python wheels + sdists; also the merge-queue build check |
| `.github/workflows/publish-c-cpp-libraries.yml` | C/C++ library archives |
| `scripts/release/verify-release.py` | Completeness gate — reads back what a release actually published |
| `scripts/qa/publish-graph.sh` | Static publish-graph rules, run in PR CI (`make qa-publish-graph`) |

### Required GitHub secrets

- `CARGO_REGISTRY_TOKEN`: crates.io API token
- `PYPI_PASS`: PyPI API token, used by `pip-release.yml` for both `dora-rs` and `dora-rs-cli`. Keyless [OIDC trusted publishing](https://docs.pypi.org/trusted-publishers/) needs a publisher registered on PyPI first; the exchange fails with `invalid-publisher` until one is. To switch, register a publisher for workflow `pip-release.yml` and environment `pypi`, then swap `MATURIN_PYPI_TOKEN` for the OIDC action.
