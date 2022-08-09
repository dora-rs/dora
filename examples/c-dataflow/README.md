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

The [`run.rs`](./run.rs) binary performs all required build steps and then starts the dataflow. It can be invoked through `cargo run --example c-dataflow`.

