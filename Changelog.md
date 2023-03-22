# Changelog

## v0.2.1 (2023-03-22)

### Features

* [Make dora-rs publishable on crates.io](https://github.com/dora-rs/dora/pull/211)
* [Add an internal cli argument to create template with path dependencies](https://github.com/dora-rs/dora/pull/212)

### Fixes

* [Avoid blocking the daemon main loop by using unbounded queue](https://github.com/dora-rs/dora/pull/230)
* [Inject YAML declared env variable into the runtime](https://github.com/dora-rs/dora/pull/227)
* [Use rustls instead of system SSL implementation](https://github.com/dora-rs/dora/pull/216)

### Other

* [Refactor python error](https://github.com/dora-rs/dora/pull/229)
* [The first letter of rust should be lowercase in the command](https://github.com/dora-rs/dora/pull/226)
* [Add documentation to the cli within the helper mode](https://github.com/dora-rs/dora/pull/225)
* [Update to safer-ffi v0.1.0-rc1](https://github.com/dora-rs/dora/pull/218)
* [remove unused variable: data_bytes](https://github.com/dora-rs/dora/pull/215)
* [Clean up: Remove workspace path](https://github.com/dora-rs/dora/pull/210)
* [Decouple opentelemetry from tracing](https://github.com/dora-rs/dora/pull/222)
* [Remove zenoh dependency from dora node API to speed up build](https://github.com/dora-rs/dora/pull/220)
* [Update to Rust v1.68](https://github.com/dora-rs/dora/pull/221)
* [Deny unknown fields to avoid typos](https://github.com/dora-rs/dora/pull/223)

## v0.2.0 (2023-03-14)

### Breaking

* [Redesign: Create a `dora-daemon` as a communication broker](https://github.com/dora-rs/dora/pull/162)
  * New `dora-daemon` executable that acts as a communication hub for all local nodes
  * Large messages are passed through shared memory without any copying
  * [Replaces the previous `iceoryx` communication layer](https://github.com/dora-rs/dora/pull/201)
  * Small API change: Nodes and operators now receive _events_ instead of just inputs
    * Inputs are one type of event
    * Other supported events: `InputClosed` when an input stream is closed and `Stop` when the user stops the dataflow (e.g. through the CLI)

### Features

* Better Error handling when operator fails
* [Send small messages directly without shared memory](https://github.com/dora-rs/dora/pull/193)
* [Send all queued incoming events at once on `NextEvent` request](https://github.com/dora-rs/dora/pull/194)
* [Don't send replies for `SendMessage` requests when using TCP](https://github.com/dora-rs/dora/pull/195)
* [Allocate shared memory in nodes to improve throughput](https://github.com/dora-rs/dora/pull/200)

### Fixes

* [Manage node failure: Await all nodes to finish before marking dataflow as finished](https://github.com/dora-rs/dora/pull/183)

### Other

* [Use `DoraStatus` from dora library in template](https://github.com/dora-rs/dora/pull/182)
* [Simplify: Replace `library_filename` function with `format!` call](https://github.com/dora-rs/dora/pull/191)
* [Refactor Rust node API implementation](https://github.com/dora-rs/dora/pull/196)
* [Remove code duplicate for tracing subscriber and use env variable to manage log level.](https://github.com/dora-rs/dora/pull/197)
* [Add daemon to the release archive](https://github.com/dora-rs/dora/pull/199)
* [Remove `remove_dir_all` from `Cargo.lock`as it is vulnerable to a race condition according to dependabot](https://github.com/dora-rs/dora/pull/202)
* [Update the documentation to the new daemon format](https://github.com/dora-rs/dora/pull/198)
* [Removing legacy `libacl` which was required by Iceoryx](https://github.com/dora-rs/dora/pull/205)
* [Remove unimplemented CLI arguments for now](https://github.com/dora-rs/dora/pull/207)
* [Update zenoh to remove git dependencies](https://github.com/dora-rs/dora/pull/203)
* [Fix cli template to new daemon API](https://github.com/dora-rs/dora/pull/204)
* [Cleanup warnings](https://github.com/dora-rs/dora/pull/208)
* Dependency updates

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
