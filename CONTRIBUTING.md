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

## Publishing new Versions

The maintainers are responsible for publishing new versions of the `adora` crates.
Please don't open unsolicited pull requests to create new releases.
Instead, request a new version by opening an issue or by leaving a comment on a merged PR.
