# Rust API

## Operator 

The operator API is a framework for you to implement. The implemented operator will be managed by `dora`. This framework enable us to make optimisation and provide advanced features. It is the recommended way of using `dora`.

An operator requires to be registered and implement the `DoraOperator` trait. It is composed of an `on_input` method that defines the behaviour of the operator when there is an input.

```rust
use dora_operator_api::{register_operator, DoraOperator, DoraOutputSender, DoraStatus};

register_operator!(ExampleOperator);

#[derive(Debug, Default)]
struct ExampleOperator {
    time: Option<String>,
}

impl DoraOperator for ExampleOperator {
    fn on_input(
        &mut self,
        id: &str,
        data: &[u8],
        output_sender: &mut DoraOutputSender,
    ) -> Result<DoraStatus, ()> {
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
let node = DoraNode::init_from_env().await?;
```

#### `.inputs()`

`.inputs()` gives you a stream of input that you can access using `next()` on the input stream.

```rust
let mut inputs = node.inputs().await?;
```

#### `.send_output(output_id, data)`

`send_output` send data from the node.

```rust
node.send_output(&data_id, data.as_bytes()).await?;
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
