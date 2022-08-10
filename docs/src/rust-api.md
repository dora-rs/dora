# Rust API

## Node

The custom node API allow you to integrate `dora` into your application. It allows you to retrieve input and send output in any fashion you want. 

### Try it out!

- Generate a new Rust binary (application):

```bash
cargo new source_timer
```

```toml
[package]
name = "source_timer"
version = "0.1.0"
edition = "2021"
license = "Apache-2.0"

[dependencies]
dora-node-api = { path = "../../apis/rust/node" }
time = "0.3.9"
```

`src/main.rs`
```rust
{{#include ../../binaries/coordinator/examples/nodes/rust/source_timer.rs}}
```

- Link it in your graph as:
```yaml
  - id: timer
    custom:
      run: cargo run --release
      outputs:
        - time
```

## Operator 

The operator API gives you a framework for operator that is going to be managed by `dora`. This framework enable us to make optimisation and provide advanced features.

### Try it out!

- Generate a new Rust library

```bash
cargo new example-operator --lib
```

`Cargo.toml`
```toml
{{#include ../../examples/example-operator/Cargo.toml}}
```

`src/lib.rs`
```rust
{{#include ../../examples/example-operator/src/lib.rs}}
```

- Build it:
```bash
cargo build --release
```

- Link it in your graph as:
```yaml
{{#include ../../binaries/coordinator/examples/mini-dataflow.yml:38:46}}
```

This example can be found in `examples`.