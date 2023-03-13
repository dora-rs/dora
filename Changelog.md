# Changelog

## v0.2.0 (2023-01-18)

* Use self-made shared memory pub sub
* Use event instead for input
* Use dora daemon to manage messages
* Better Error handling when operator fails

## v0.1.3 (2023-01-18)

* Package `DoraStatus` into dora python package: https://github.com/dora-rs/dora/pull/172
* Force removal of Pyo3 Object to avoid memory leak: https://github.com/dora-rs/dora/pull/168
* Bump tokio from 1.21.2 to 1.23.1: https://github.com/dora-rs/dora/pull/171
* Create a changelog file: https://github.com/dora-rs/dora/pull/174 

## v0.1.2 (2022-12-15)

- Fix infinite loop in the coordinator: https://github.com/dora-rs/dora/pull/155
- Simplify the release process: https://github.com/dora-rs/dora/pull/157
- Use generic linux distribution: https://github.com/dora-rs/dora/pull/159

## v0.1.1 (2022-12-05)

This release contains fixes for:
- Python linking using pypi release but also a redesigned python thread model within the runtime to avoid deadlock of the `GIL`. This also fix an issue with `patchelf`.
- A deployment separation for `ubuntu` as the `20.04` version of `dora` and `22.04` version of dora are non-compatible.
- A better tagging of api for `dora` Rust API.

## v0.1.0 (2022-11-15)

This is our first release of `dora-rs`!

The current release includes:
- `dora-cli` which enables creating, starting and stopping dataflow.
- `dora-coordinator` which is our control plane.
- `dora-runtime` which is manage the runtime of operators.
- `custom-nodes` API which enables bridges from different languages.
