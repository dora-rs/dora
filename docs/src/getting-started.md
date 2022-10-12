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
    "rust-dataflow-example-node",
]
```  

### Write your first node

Let's write a node which sends the current time periodically. Let's make it after 100 iterations. The other nodes/operators will then exit as well because all sources closed.

- Generate a new Rust binary (application):

```bash
cargo new rust-dataflow-example-node
```

with `Cargo.toml`:
```toml
{{#include ../../examples/rust-dataflow/node/Cargo.toml}}
```

with `src/main.rs`:
```rust
{{#include ../../examples/rust-dataflow/node/src/main.rs}}
```

### Write your first operator 

- Generate a new Rust library:

```bash
cargo new rust-dataflow-example-operator --lib
```

with `Cargo.toml`:
```toml
{{#include ../../examples/rust-dataflow/operator/Cargo.toml}}
```

with `src/lib.rs`:
```rust
{{#include ../../examples/rust-dataflow/operator/src/lib.rs}}
```

- And modify the root `Cargo.toml`:
```toml=
[workspace]

members = [
    "rust-dataflow-example-node",
    "rust-dataflow-example-operator",
]
```



### Write your sink node 

Let's write a `logger` which will print incoming data.

- Generate a new Rust binary (application):

```bash
cargo new sink_logger
```

with `Cargo.toml`:
```toml
{{#include ../../examples/rust-dataflow/sink/Cargo.toml}}
```

with `src/main.rs`:
```rust
{{#include ../../examples/rust-dataflow/sink/src/main.rs}}
```

- And modify the root `Cargo.toml`:
```toml=
[workspace]

members = [
    "rust-dataflow-example-node",
    "rust-dataflow-example-operator",
    "rust-dataflow-example-sink"
]
```

### Compile everything

```bash
cargo build --all --release
```


### Write a graph definition

Let's write the graph definition so that the nodes know who to communicate with.

`dataflow.yml`
```yaml
{{#include ../../examples/rust-dataflow/dataflow.yml}}
```

### Run it!

- Run the `dataflow`: 
```bash 
dora-coordinator --run-dataflow dataflow.yml dora-runtime
```
