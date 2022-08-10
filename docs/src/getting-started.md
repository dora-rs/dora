### Create a Rust workspace

- Initiate the workspace with:

```bash
mkdir my_first_dataflow
cd my_first_dataflow
```

- Create the Cargo.toml file that will configure the entire workspace:

`Cargo.toml`
```toml
[workspace]

members = [
    "source_timer",
]
```  

### Write your first node

Let's write a node which sends the current time periodically. Let's make it after 100 iterations. The other nodes/operators will then exit as well because all sources closed.

- Generate a new Rust binary (application):

```bash
cargo new source_timer
```

with `Cargo.toml`:
```toml
[package]
name = "rust-node"
version = "0.1.0"
edition = "2021"
license = "Apache-2.0"

[dependencies]
dora-node-api = { git = "https://github.com/dora-rs/dora" }
time = "0.3.9"
```

with `src/main.rs`:
```rust
{{#include ../../binaries/coordinator/examples/nodes/rust/source_timer.rs}}
```

### Write your second node 

Let's write a `logger` which will print incoming data.

- Generate a new Rust binary (application):

```bash
cargo new sink_logger
```

with `Cargo.toml`:
```toml
[package]
name = "sink_logger"
version = "0.1.0"
edition = "2021"
license = "Apache-2.0"

[dependencies]
dora-node-api = { git = "https://github.com/dora-rs/dora" }
time = "0.3.9"
```

with `src/main.rs`:
```rust
{{#include ../../binaries/coordinator/examples/nodes/rust/sink_logger.rs}}
```

- And modify the root `Cargo.toml`:
```toml=
[workspace]

members = [
    "source_timer",
    "sink_logger"
]
```


### Write a graph definition

Let's write the graph definition so that the nodes know who to communicate with.

`mini-dataflow.yml`
```yaml
communication:
  zenoh:
    prefix: /foo

nodes:
  - id: timer
    custom:
      run: cargo run --release --bin source_timer
      outputs:
        - time

  - id: logger
    custom:
      run: cargo run --release --bin sink_logger
      inputs:
        time: timer/time
```

### Run it!

- Run the `mini-dataflow`: 
```bash 
dora-coordinator run mini-dataflow.yml
```
