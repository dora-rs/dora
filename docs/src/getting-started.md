### 2. Compile the Rust example operator:

```bash
cargo build --manifest-path ../examples/example-operator/Cargo.toml --release
```

### 3. Compile the C example operator:
```bash
cd ../../examples/c-operator
cp ../../apis/c/operator/api.h .
clang -c operator.c
clang -shared -v operator.o -o operator.so
```
- Run the `mini-dataflow` example using `cargo run --release -- run examples/mini-dataflow.yml`
  - This spawns a `timer` source, which sends the current time periodically, and a `logger` sink, which prints the incoming data.
  - The `timer` will exit after 100 iterations. The other nodes/operators will then exit as well because all sources closed.

