# How to contribute to `adora-rs`

We welcome bug reports, feature requests, and pull requests!

Please discuss non-trivial changes in a Github issue or on Discord first before implementing them.
This way, we can avoid unnecessary work on both sides.

## Building

The `adora` project is set up as a [cargo workspace](https://doc.rust-lang.org/cargo/reference/workspaces.html).
You can use the standard `cargo check`, `cargo build`, `cargo run`, and `cargo test` commands.
To run a command for a specific package only, pass e.g. `--package adora-daemon`.
Running a command for the whole workspace is possible by passing `--workspace`.

```bash
# Build all (excluding Python packages which require maturin)
cargo build --all --exclude adora-node-api-python --exclude adora-operator-api-python --exclude adora-ros2-bridge-python

# Test all
cargo test --all --exclude adora-node-api-python --exclude adora-operator-api-python --exclude adora-ros2-bridge-python

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

### Adora Bot

We use a custom Github Action to help manage issue assignments. You can interact with this action using the following:

- `@adora-bot assign me` - Assigns the current issue to you.
- `@adora-bot unassign me` - Removes yourself from the issue assignment.

For maintainers only:
- `adora-bot unassign @username` - Allows maintainers to unassign other contributors
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

CI takes over from the tag push and automatically:
- Publishes all crates to crates.io (in dependency order)
- Builds and publishes the Python node API wheel to PyPI (`adora-rs`)
- Builds CLI binaries for all platforms
- Creates a GitHub Release with changelog and binary assets

### Configuration

| File | Purpose |
|------|---------|
| `release.toml` | cargo-release settings (version bump, tagging) |
| `cliff.toml` | git-cliff changelog generation |
| `.github/workflows/release.yml` | Tag-triggered publish pipeline |

### Required GitHub secrets

- `CARGO_REGISTRY_TOKEN`: crates.io API token
- PyPI: configure [OIDC trusted publishing](https://docs.pypi.org/trusted-publishers/) for the `pypi` environment (no secret needed)
