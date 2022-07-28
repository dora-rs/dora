# Coordinator

Prototype for a process/library-based dora-rs implementation, instead of framework-based. The idea is that each operator is compiled as a separate executable. The `dora-coordinator` runtime is responsible for reading the dataflow descriptor file and launching the operators accordingly. The operators use a common library called `dora-api`, which implements the communication layer based on zenoh.

This approach has the following advantages:

- Less overhead
  - No data transfer between a runtime and the operator
  - The compiler can inline and optimize the full process
- More flexibility
  - Operators can be sync or async
  - They can decide how many threads and which execution model they use
  - The OS ensures fair share of resources (e.g. CPU time) -> no need to cooperate with other operators
  - Operators get all inputs immediately -> no need for input rules
  - Keeping local state is easily possible
- Separate address spaces
  - The operators are isolated from each other.

There are drawbacks too, for example:

- Less control
  - Processes run independently -> need to cooperate with the runtime, e.g. on stop signals
  - Operator migration is more difficult
- Operators are always isolated
  - No way of using in-memory channels
  - Local sockets and shared memory should be still possible

## Try it out

- Compile: the dora runtime,  the nodes in `examples`, and the Rust example operator through:
```bash
cargo build -p dora-runtime --release
cargo build -p dora-coordinator --examples --release
cargo build --manifest-path ../examples/example-operator/Cargo.toml --release
```
- Compile the C example operator through:
```bash
cd ../../examples/c-operator
cp ../../apis/c/operator/api.h .
clang -c operator.c
clang -shared -v operator.o -o operator.so
```
- Run the `mini-dataflow` example using `cargo run --release -- run examples/mini-dataflow.yml`
  - This spawns a `timer` source, which sends the current time periodically, and a `logger` sink, which prints the incoming data.
  - The `timer` will exit after 100 iterations. The other nodes/operators will then exit as well because all sources closed.

