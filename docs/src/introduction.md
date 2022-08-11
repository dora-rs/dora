# Welcome to `dora`!

`dora` goal is to be a low latency, composable, and distributed data flow.

By using `dora`, you can define robotic applications as a graph of nodes that can be easily swapped and replaced. Those nodes can be shared and implemented in different languages such as Rust, Python or C. `dora` will then connect those nodes and try to provide as many features as possible to facilitate the dataflow.

## ✨ Features that we want to provide

Composability as:
- [x] `YAML` declarative programming
- [x] language-agnostic:
  - [x] Rust
  - [x] C
  - [x] Python
- [ ] Isolated operators and nodes that can be reused.

Low latency as:
- [x] written in  <i>...Cough...blazingly fast ...Cough...</i> Rust.
- [ ] Minimal abstraction, close to the metal.

Distributed as:
- [x] PubSub communication with [`zenoh`](https://github.com/eclipse-zenoh/zenoh)
- [x] Distributed telemetry with [`opentelemetry`](https://github.com/open-telemetry/opentelemetry-rust)



## ⚖️ LICENSE 

This project is licensed under Apache-2.0. Check out [NOTICE.md](NOTICE.md) for more information.

