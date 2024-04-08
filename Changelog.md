# Changelog

## v0.3.3 (2024-04-08)

## What's Changed

- Metrics refactoring by @haixuanTao in https://github.com/dora-rs/dora/pull/423
- Add ROS2 bridge support for C++ nodes by @phil-opp in https://github.com/dora-rs/dora/pull/425
- Provide function to create empty `CombinedEvents` stream by @phil-opp in https://github.com/dora-rs/dora/pull/432
- Expose ROS2 constants in generated bindings (Rust and C++) by @phil-opp in https://github.com/dora-rs/dora/pull/428
- Add option to send `stdout` as node/operator output by @haixuanTao in https://github.com/dora-rs/dora/pull/388
- Fix warning about `#pragma once` in main file by @phil-opp in https://github.com/dora-rs/dora/pull/433
- Send runs artefacts into a dedicated `out` folder by @haixuanTao in https://github.com/dora-rs/dora/pull/429
- Create README.md for cxx-ros2-example by @bobd988 in https://github.com/dora-rs/dora/pull/431
- Use Async Parquet Writer for `dora-record` by @haixuanTao in https://github.com/dora-rs/dora/pull/434
- Update mio to fix security vulnerability by @phil-opp in https://github.com/dora-rs/dora/pull/440
- Add initial support for calling ROS2 services from Rust nodes by @phil-opp in https://github.com/dora-rs/dora/pull/439
- Enable ROS2 service calls from C++ nodes by @phil-opp in https://github.com/dora-rs/dora/pull/441
- Use `Debug` formatting for eyre errors when returning to C++ by @phil-opp in https://github.com/dora-rs/dora/pull/450
- Fix out-of-tree builds in cmake example by @phil-opp in https://github.com/dora-rs/dora/pull/453
- Llm example by @haixuanTao in https://github.com/dora-rs/dora/pull/451

## Minor

- Bump h2 from 0.3.24 to 0.3.26 by @dependabot in https://github.com/dora-rs/dora/pull/456
- Update `bat` dependency to v0.24 by @phil-opp in https://github.com/dora-rs/dora/pull/424

## New Contributors

- @bobd988 made their first contribution in https://github.com/dora-rs/dora/pull/431

**Full Changelog**: https://github.com/dora-rs/dora/compare/v0.3.2...v0.3.3

## v0.3.2 (2024-01-26)

## Features

- Wait until `DestroyResult` is sent before exiting dora-daemon by @phil-opp in https://github.com/dora-rs/dora/pull/413
- Reduce dora-rs to a single binary by @haixuanTao in https://github.com/dora-rs/dora/pull/410
- Rework python ROS2 (de)serialization using parsed ROS2 messages directly by @phil-opp in https://github.com/dora-rs/dora/pull/415
- Fix ros2 array bug by @haixuanTao in https://github.com/dora-rs/dora/pull/412
- Test ros2 type info by @haixuanTao in https://github.com/dora-rs/dora/pull/418
- Use forward slash as it is default way of defining ros2 topic by @haixuanTao in https://github.com/dora-rs/dora/pull/419

## Minor

- Bump h2 from 0.3.21 to 0.3.24 by @dependabot in https://github.com/dora-rs/dora/pull/414

## v0.3.1 (2024-01-09)

## Features

- Support legacy python by @haixuanTao in https://github.com/dora-rs/dora/pull/382
- Add an error catch in python `on_event` when using hot-reloading by @haixuanTao in https://github.com/dora-rs/dora/pull/372
- add cmake example by @XxChang in https://github.com/dora-rs/dora/pull/381
- Bump opentelemetry metrics to 0.21 by @haixuanTao in https://github.com/dora-rs/dora/pull/383
- Trace send_output as it can be a big source of overhead for large messages by @haixuanTao in https://github.com/dora-rs/dora/pull/384
- Adding a timeout method to not block indefinitely next event by @haixuanTao in https://github.com/dora-rs/dora/pull/386
- Adding `Vec<u8>` conversion by @haixuanTao in https://github.com/dora-rs/dora/pull/387
- Dora cli renaming by @haixuanTao in https://github.com/dora-rs/dora/pull/399
- Update `ros2-client` and `rustdds` dependencies to latest fork version by @phil-opp in https://github.com/dora-rs/dora/pull/397

## Fix

- Fix window path error by @haixuanTao in https://github.com/dora-rs/dora/pull/398
- Fix read error in C++ node input by @haixuanTao in https://github.com/dora-rs/dora/pull/406
- Bump unsafe-libyaml from 0.2.9 to 0.2.10 by @dependabot in https://github.com/dora-rs/dora/pull/400

## New Contributors

- @XxChang made their first contribution in https://github.com/dora-rs/dora/pull/381

**Full Changelog**: https://github.com/dora-rs/dora/compare/v0.3.0...v0.3.1

## v0.3.0 (2023-11-01)

## Features

- Rust node API typed using arrow by @phil-opp in https://github.com/dora-rs/dora/pull/353
- Dora record by @haixuanTao in https://github.com/dora-rs/dora/pull/365
- beautify graph visualisation by @haixuanTao in https://github.com/dora-rs/dora/pull/370
- Remove `Ros2Value` encapsulation of `ArrayData` by @haixuanTao in https://github.com/dora-rs/dora/pull/359
- Refactor python typing by @haixuanTao in https://github.com/dora-rs/dora/pull/369
- Update README discord link by @Felixhuangsiling in https://github.com/dora-rs/dora/pull/361

### Other

- Update `rustix` v0.38 dependency by @phil-opp in https://github.com/dora-rs/dora/pull/366
- Bump rustix from 0.37.24 to 0.37.25 by @dependabot in https://github.com/dora-rs/dora/pull/364
- Bump quinn-proto from 0.9.3 to 0.9.5 by @dependabot in https://github.com/dora-rs/dora/pull/357
- Bump webpki from 0.22.1 to 0.22.2 by @dependabot in https://github.com/dora-rs/dora/pull/358
- Update README discord link by @Felixhuangsiling in https://github.com/dora-rs/dora/pull/361

## New Contributors

- @Felixhuangsiling made their first contribution in https://github.com/dora-rs/dora/pull/361

## v0.2.6 (2023-09-14)

- Update dependencies to fix some security advisories by @phil-opp in https://github.com/dora-rs/dora/pull/354
  - Fixes `cargo install dora-daemon`

## v0.2.5 (2023-09-06)

### Features

- Use cargo instead of git in Rust `Cargo.toml` template by @haixuanTao in https://github.com/dora-rs/dora/pull/326
- Use read_line instead of next_line in stderr by @haixuanTao in https://github.com/dora-rs/dora/pull/325
- Add a `rust-ros2-dataflow` example using the dora-ros2-bridge by @phil-opp in https://github.com/dora-rs/dora/pull/324
- Removing patchelf by @haixuanTao in https://github.com/dora-rs/dora/pull/333
- Improving python example readibility by @haixuanTao in https://github.com/dora-rs/dora/pull/334
- Use `serde_bytes` to serialize `Vec<u8>` by @haixuanTao in https://github.com/dora-rs/dora/pull/336
- Adding support for `Arrow List(*)` for Python by @haixuanTao in https://github.com/dora-rs/dora/pull/337
- Bump rustls-webpki from 0.100.1 to 0.100.2 by @dependabot in https://github.com/dora-rs/dora/pull/340
- Add support for event stream merging for Python node API by @phil-opp in https://github.com/dora-rs/dora/pull/339
- Merge `dora-ros2-bridge` by @phil-opp in https://github.com/dora-rs/dora/pull/341
- Update dependencies by @phil-opp in https://github.com/dora-rs/dora/pull/345
- Add support for arbitrary Arrow types in Python API by @phil-opp in https://github.com/dora-rs/dora/pull/343
- Use typed inputs in Python ROS2 example by @phil-opp in https://github.com/dora-rs/dora/pull/346
- Use struct type instead of array for ros2 messages by @haixuanTao in https://github.com/dora-rs/dora/pull/349

### Other

- Add Discord :speech_balloon: by @haixuanTao in https://github.com/dora-rs/dora/pull/348
- Small refactoring by @haixuanTao in https://github.com/dora-rs/dora/pull/342

## v0.2.4 (2023-07-18)

### Features

- Return dataflow result to CLI on `dora stop` by @phil-opp in https://github.com/dora-rs/dora/pull/300
- Make dataflow descriptor available to Python nodes and operators by @phil-opp in https://github.com/dora-rs/dora/pull/301
- Create a `CONTRIBUTING.md` guide by @phil-opp in https://github.com/dora-rs/dora/pull/307
- Distribute prebuilt arm macos dora-rs by @haixuanTao in https://github.com/dora-rs/dora/pull/308

### Other

- Fix the typos and add dora code branch by @meua in https://github.com/dora-rs/dora/pull/290
- For consistency with other examples, modify python -> python3 by @meua in https://github.com/dora-rs/dora/pull/299
- Add timestamps generated by hybrid logical clocks to all sent events by @phil-opp in https://github.com/dora-rs/dora/pull/302
- Don't recompile the `dora-operator-api-c` crate on every build/run by @phil-opp in https://github.com/dora-rs/dora/pull/304
- Remove deprecated `proc_macros` feature from `safer-ffi` dependency by @phil-opp in https://github.com/dora-rs/dora/pull/305
- Update to Rust v1.70 by @phil-opp in https://github.com/dora-rs/dora/pull/303
- Fix issue with not finding a custom nodes path by @haixuanTao in https://github.com/dora-rs/dora/pull/315
- Implement `Stream` for `EventStream` by @phil-opp in https://github.com/dora-rs/dora/pull/309
- Replace unmaintained `atty` crate with `std::io::IsTerminal` by @phil-opp in https://github.com/dora-rs/dora/pull/318

**Full Changelog**: https://github.com/dora-rs/dora/compare/v0.2.3...v0.2.4

## v0.2.3 (2023-05-24)

## What's Changed

- Check that coordinator, daemon, and node versions match by @phil-opp in https://github.com/dora-rs/dora/pull/245
- Share events to Python without copying via `arrow` crate by @phil-opp in https://github.com/dora-rs/dora/pull/228
- Upgrading the operator example to use `dora-arrow` by @haixuanTao in https://github.com/dora-rs/dora/pull/251
- [Python] Show node name in process and put Traceback before the actual Error for more natural error by @haixuanTao in https://github.com/dora-rs/dora/pull/255
- CLI: Improve error messages when coordinator is not running by @phil-opp in https://github.com/dora-rs/dora/pull/254
- Integrate `dora-runtime` into `dora-daemon` by @phil-opp in https://github.com/dora-rs/dora/pull/257
- Filter default log level at `warn` for `tokio::tracing` by @haixuanTao in https://github.com/dora-rs/dora/pull/269
- Make log level filtering be `WARN` or below by @haixuanTao in https://github.com/dora-rs/dora/pull/274
- Add support for distributed deployments with multiple daemons by @phil-opp in https://github.com/dora-rs/dora/pull/256
- Provide a way to access logs through the CLI by @haixuanTao in https://github.com/dora-rs/dora/pull/259
- Handle node errors during initialization phase by @phil-opp in https://github.com/dora-rs/dora/pull/275
- Replace watchdog by asynchronous heartbeat messages by @phil-opp in https://github.com/dora-rs/dora/pull/278
- Remove pyo3 in runtime and daemon as it generates `libpython` depende… by @haixuanTao in https://github.com/dora-rs/dora/pull/281
- Release v0.2.3 with aarch64 support by @haixuanTao in https://github.com/dora-rs/dora/pull/279

## Fix

- Fix yolov5 dependency issue by @haixuanTao in https://github.com/dora-rs/dora/pull/291
- To solve this bug https://github.com/dora-rs/dora/issues/283, unify t… by @meua in https://github.com/dora-rs/dora/pull/285
- Fix: Don't try to create two global tracing subscribers when using bundled runtime by @phil-opp in https://github.com/dora-rs/dora/pull/277
- CI: Increase timeout for 'build CLI and binaries' step by @phil-opp in https://github.com/dora-rs/dora/pull/282

## Other

- Update `pyo3` to `v0.18` by @phil-opp in https://github.com/dora-rs/dora/pull/246
- Bump h2 from 0.3.13 to 0.3.17 by @dependabot in https://github.com/dora-rs/dora/pull/249
- Add automatic issue labeler to organize opened issues by @haixuanTao in https://github.com/dora-rs/dora/pull/265
- Allow the issue labeler to write issues by @phil-opp in https://github.com/dora-rs/dora/pull/272
- Add a support matrix with planned feature to clarify dora status by @haixuanTao in https://github.com/dora-rs/dora/pull/264

**Full Changelog**: https://github.com/dora-rs/dora/compare/v0.2.2...v0.2.3

## v0.2.2 (2023-04-01)

### Features

- Make queue length configurable through the dataflow file by @phil-opp in https://github.com/dora-rs/dora/pull/231
- Hot reloading Python Operator by @haixuanTao in https://github.com/dora-rs/dora/pull/239
- Synchronize node and operator start by @phil-opp in https://github.com/dora-rs/dora/pull/236
- Add opentelemetry capability at runtime instead of compile time by @haixuanTao in https://github.com/dora-rs/dora/pull/234

### Others

- Wait on events and messages simultaneously to prevent queue buildup by @phil-opp in https://github.com/dora-rs/dora/pull/235
- Fix looping in daemon listener loop by @phil-opp in https://github.com/dora-rs/dora/pull/244
- Validate shell command as source and url source by @haixuanTao in https://github.com/dora-rs/dora/pull/243
- Push error into the `init_done` channel for debugging context by @haixuanTao in https://github.com/dora-rs/dora/pull/238
- Option communication config by @haixuanTao in https://github.com/dora-rs/dora/pull/241
- Validate yaml when reading by @haixuanTao in https://github.com/dora-rs/dora/pull/237

**Full Changelog**: https://github.com/dora-rs/dora/compare/v0.2.1...v0.2.2

## v0.2.1 (2023-03-22)

### Features

- [Make dora-rs publishable on crates.io](https://github.com/dora-rs/dora/pull/211)

### Fixes

- [Avoid blocking the daemon main loop by using unbounded queue](https://github.com/dora-rs/dora/pull/230)
- [Inject YAML declared env variable into the runtime](https://github.com/dora-rs/dora/pull/227)
- [Use rustls instead of system SSL implementation](https://github.com/dora-rs/dora/pull/216)

### Other

- [Refactor python error](https://github.com/dora-rs/dora/pull/229)
- [The first letter of rust should be lowercase in the command](https://github.com/dora-rs/dora/pull/226)
- [Add documentation to the cli within the helper mode](https://github.com/dora-rs/dora/pull/225)
- [Update to safer-ffi v0.1.0-rc1](https://github.com/dora-rs/dora/pull/218)
- [remove unused variable: data_bytes](https://github.com/dora-rs/dora/pull/215)
- [Clean up: Remove workspace path](https://github.com/dora-rs/dora/pull/210)
- [Decouple opentelemetry from tracing](https://github.com/dora-rs/dora/pull/222)
- [Remove zenoh dependency from dora node API to speed up build](https://github.com/dora-rs/dora/pull/220)
- [Update to Rust v1.68](https://github.com/dora-rs/dora/pull/221)
- [Deny unknown fields to avoid typos](https://github.com/dora-rs/dora/pull/223)
- [Add an internal cli argument to create template with path dependencies](https://github.com/dora-rs/dora/pull/212)

## v0.2.0 (2023-03-14)

### Breaking

- [Redesign: Create a `dora-daemon` as a communication broker](https://github.com/dora-rs/dora/pull/162)
  - New `dora-daemon` executable that acts as a communication hub for all local nodes
  - Large messages are passed through shared memory without any copying
  - [Replaces the previous `iceoryx` communication layer](https://github.com/dora-rs/dora/pull/201)
  - Small API change: Nodes and operators now receive _events_ instead of just inputs
    - Inputs are one type of event
    - Other supported events: `InputClosed` when an input stream is closed and `Stop` when the user stops the dataflow (e.g. through the CLI)

### Features

- Better Error handling when operator fails
- [Send small messages directly without shared memory](https://github.com/dora-rs/dora/pull/193)
- [Send all queued incoming events at once on `NextEvent` request](https://github.com/dora-rs/dora/pull/194)
- [Don't send replies for `SendMessage` requests when using TCP](https://github.com/dora-rs/dora/pull/195)
- [Allocate shared memory in nodes to improve throughput](https://github.com/dora-rs/dora/pull/200)

### Fixes

- [Manage node failure: Await all nodes to finish before marking dataflow as finished](https://github.com/dora-rs/dora/pull/183)

### Other

- [Use `DoraStatus` from dora library in template](https://github.com/dora-rs/dora/pull/182)
- [Simplify: Replace `library_filename` function with `format!` call](https://github.com/dora-rs/dora/pull/191)
- [Refactor Rust node API implementation](https://github.com/dora-rs/dora/pull/196)
- [Remove code duplicate for tracing subscriber and use env variable to manage log level.](https://github.com/dora-rs/dora/pull/197)
- [Add daemon to the release archive](https://github.com/dora-rs/dora/pull/199)
- [Remove `remove_dir_all` from `Cargo.lock`as it is vulnerable to a race condition according to dependabot](https://github.com/dora-rs/dora/pull/202)
- [Update the documentation to the new daemon format](https://github.com/dora-rs/dora/pull/198)
- [Removing legacy `libacl` which was required by Iceoryx](https://github.com/dora-rs/dora/pull/205)
- [Remove unimplemented CLI arguments for now](https://github.com/dora-rs/dora/pull/207)
- [Update zenoh to remove git dependencies](https://github.com/dora-rs/dora/pull/203)
- [Fix cli template to new daemon API](https://github.com/dora-rs/dora/pull/204)
- [Cleanup warnings](https://github.com/dora-rs/dora/pull/208)
- Dependency updates

## v0.1.3 (2023-01-18)

- Package `DoraStatus` into dora python package: https://github.com/dora-rs/dora/pull/172
- Force removal of Pyo3 Object to avoid memory leak: https://github.com/dora-rs/dora/pull/168
- Bump tokio from 1.21.2 to 1.23.1: https://github.com/dora-rs/dora/pull/171
- Create a changelog file: https://github.com/dora-rs/dora/pull/174

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
