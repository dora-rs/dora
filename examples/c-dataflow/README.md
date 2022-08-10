# C Dataflow Example

This examples shows how to create and connect dora operators and custom nodes in C.

## Overview

The [`dataflow.yml`](./dataflow.yml) defines a simple dataflow graph with the following three nodes:

- [`node.c`](./node.c) is a custom node, i.e., it has its own main function and runs as a separate process. It uses the [`dora-node-api-c` crate](../../apis/c/node/) to interact with the dora dataflow.
  - The node has a single input named `timer` that is mapped to a dora-provided periodic timer (`dora/timer/secs/1`).
  - Whenever the node receives a timer tick, it sends out a message with ID `tick` and a counter value as data (just a single byte).
  - After receiving 10 timer inputs, the node exits.
- The [`operator.c](./operator.c) file defines a dora _operator_ that is plugged as a shared library into a dora runtime. Instead of defining a `main` function, it implements a template of `dora_*` functions, which are invoked by the dora runtime, e.g. when new input is available.
  - The operator takes the `tick` messages created by the `node.c` node as input. For each input value, it checks the ID and then prints the received message to `stdout`.
  - It counts the received values and outputs a string of the format _"The current counter value is ..."_.
- The [`sink.c`](./sink.c) file defines a custom node again, which takes the output string of the operator as input. It prints each received input to stdout and exits as soon as the input stream is closed.

## Compile and Run

To try it out, you can use the [`run.rs`](./run.rs) binary. It performs all required build steps and then starts the dataflow. Use the following command to run it: `cargo run --example c-dataflow`.

For a manual build, follow these steps:

**Build the custom nodes:**

- Create a `build` folder in this directory (i.e., next to the `node.c` file)
- Copy the `node-api.h` header file from `../../apis/c/node` to `build`
- Compile the `dora-node-api-c` crate into a static library.
  - Run `cargo build -p dora-node-api-c --release`
  - The resulting staticlib is then available under `../../target/release/libdora-node-api-c.a`.
- Compile the `node.c` (e.g. using `clang`) and link the staticlib
  - For example, use the following command:
    ```
    clang node.c -lm -lrt -ldl -pthread -ldora_node_api_c -L ../../target/release --output build/c_node
    ```
- Repeat the previous step for the `sink.c` executable

**Build the operator:**

- Copy the `operator-api.h` header file from `../../apis/c/operator` to `build`
- Compile the `operator.c` file into a shared library.
  - For example, use the following commands:
    ```
    clang -c operator.c -o build/operator.o -fPIC
    clang -shared build/operator.o -o build/operator.so
    ```

**Build the dora coordinator and runtime:**

- Build the `dora-coordinator` executable using `cargo build -p dora-coordinator --release`
- Build the `dora-runtime` executable using `cargo build -p dora-runtime --release`

**Run the dataflow:**

- Start the `dora-coordinator`, passing the paths to the dataflow file and the `dora-runtime` as arguments:

  ```
  ../../target/release/dora-coordinator run dataflow.yml ../../target/release/dora-runtime
  ```
