# Rust API

## Operator 

The operator API is a framework for you to implement. The implemented operator will be managed by `dora`. This framework enable us to make optimisation and provide advanced features. It is the recommended way of using `dora`.

An operator requires to be registered and implement the `DoraOperator` trait. It is composed of an `on_event` method that defines the behaviour of the operator when there is an event such as receiving an input for example.

```rust
{{#include ../../examples/rust-dataflow/operator/src/lib.rs:0:17}}
```

### Try it out!

- Generate a new Rust library

```bash
cargo new rust-dataflow-example-operator --lib
```

`Cargo.toml`
```toml
{{#include ../../examples/rust-dataflow/operator/Cargo.toml}}
```

`src/lib.rs`
```rust
{{#include ../../examples/rust-dataflow/operator/src/lib.rs}}
```

- Build it:
```bash
cargo build --release
```

- Link it in your graph as:
```yaml
{{#include ../../examples/rust-dataflow/dataflow.yml:13:21}}
```

This example can be found in `examples`.

## Custom Node

The custom node API allow you to integrate `dora` into your application. It allows you to retrieve input and send output in any fashion you want. 
#### `DoraNode::init_from_env()`

`DoraNode::init_from_env()` initiate a node from environment variables set by `dora-coordinator` 

```rust
let (mut node, mut events) = DoraNode::init_from_env()?;
```

#### `.recv()`

`.recv()` wait for the next event on the events stream.

```rust
let event = events.recv();
```

#### `.send_output(...)`

`send_output` send data from the node to the other nodes.
We take a closure as an input to enable zero copy on send.

```rust
node.send_output(
    &data_id, 
    metadata.parameters,
    data.len(),
    |out| {
        out.copy_from_slice(data);
    })?;
```

### Try it out!

- Generate a new Rust binary (application):

```bash
cargo new rust-dataflow-example-node
```

```toml
{{#include ../../examples/rust-dataflow/node/Cargo.toml}}
```

`src/main.rs`
```rust
{{#include ../../examples/rust-dataflow/node/src/main.rs}}
```

- Link it in your graph as:
```yaml
{{#include ../../examples/rust-dataflow/dataflow.yml:6:12}}
```
