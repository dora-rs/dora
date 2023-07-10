# How to contribute to `dora-rs`

We welcome bug reports, feature requests, and pull requests!

Please discuss non-trivial changes in an issue or discussion first before implementing them.
This way, we can avoid unnecessary work on both sides.

## Building

The `dora` project is set up as a [cargo workspace](https://doc.rust-lang.org/cargo/reference/workspaces.html).
You can use the standard `cargo check`, `cargo build`, `cargo run`, and `cargo test` commands.
To run a command for a specific package only, pass e.g. `--package dora-daemon`.
Running a command for the whole workspace is possible by passing `--workspace`.

## Continuous Integration (CI)

We're using [GitHub Actions](https://github.com/features/actions) to run automated checks on all commits and pull requests.
These checks ensure that our `main` branch always builds successfully and that it passes all tests.
Please ensure that your pull request passes all checks.
You don't need to fix warnings that are unrelated to your changes.
Feel free to ask for help if you're unsure about a check failure.

We're currently running the following kind of checks:

- **CI / Test:** Ensures that the project builds and that all unit tests pass. This check is run on Linux, Windows, and macOS.
- **CI / Examples:** Builds and runs the Rust, C, and C++ dataflows from the `examples` subdirectory. This check is run on Linux, Windows, and macOS.
- **CI-python / Python Examples:** Builds and runs the Python dataflows from the `examples` subdirectory. This check is run on Linux only.
- **github pages / deploy:** Generates our website from the `docs` subfolder.
- **CI / CLI Test:** Runs some basic tests of the `dora` command-line application. This check is run on Linux, Windows, and macOS.
- **CI / Clippy:** Runs the additional checks of the [`clippy`](https://github.com/rust-lang/rust-clippy) project.
- **CI / Formatting:** Ensures that the code is formatted using `rustfmt` (see [below](#style))
- **CI / License Checks:** Scans the dependency tree and tries to detect possible license incompatibilities.

## Style

We use [`rustfmt`](https://github.com/rust-lang/rustfmt) with its default settings to format our code.
Please run `cargo fmt --all` on your code before submitting a pull request.
Our CI will run an automatic formatting check of your code.

## Publishing new Versions

The maintainers are responsible for publishing new versions of the `dora` crates.
Please don't open unsolicited pull requests to create new releases.
Instead, request a new version by opening an issue or by leaving a comment on a merged PR.
