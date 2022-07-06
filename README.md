<p align="center">
    <img src="./design/logo.svg" width="400">
</p>

<h3 align="center">
Dataflow Oriented Robotic Architecture ⚡
</h3>

---

`dora` goal is to be a low latency, composable, and distributed data flow.

This project is in early development, and many features have yet to be implemented with breaking changes. Please don't take for granted the current design.

---
## ✨ Features

Composability as:
- [x] `YAML` declarative programming
- [ ] language-agnostic:
  - [x] Rust
  - [x] C
  - [ ] Python
- [ ] Isolated operator and node that can be reused.

Low latency as:
- [x] written in  <i>...Cough...blazingly fast ...Cough...</i> Rust.
- [ ] Minimal abstraction close to the metal.

Distributed as:
- [x] PubSub communication with [`zenoh`](https://github.com/eclipse-zenoh/zenoh)
- [x] Distributed telemetry with [`opentelemetry`](https://github.com/open-telemetry/opentelemetry-rust)

---

## 💻 Getting started

- Check out [coordinator/README.md](coordinator/README.md)

## 🏁 Further reading

- Check out [dora-drives](https://github.com/dora-rs/dora-drives) for a template of an autonomous vehicle within a simulation.


## ⚖️ LICENSE 

This project is licensed under Apache-2.0. Check out [NOTICE.md](NOTICE.md) for more information.
