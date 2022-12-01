<p align="center">
    <img src="./docs/src/logo.svg" width="400">
</p>

<h3 align="center">
Dataflow Oriented Robotic Architecture ⚡
</h3>

---

`dora` goal is to be a low latency, composable, and distributed data flow.

This project is in early development, and many features have yet to be implemented with breaking changes. Please don't take for granted the current design.

---
## 📖 Documentation

The documentation can be found here: [https://dora-rs.github.io/dora/](https://dora-rs.github.io/dora/) 

---

## Getting started


1. Install `dora` binaries from GitHub releases

For linux
```bash
wget https://github.com/dora-rs/dora/releases/download/<version>/dora-<version>-x86_64-Linux.zip
unzip dora-<version>-x86_64-Linux.zip
python3 -m pip install dora-rs==<version>
PATH=$PATH:$(pwd):$(pwd)/iceoryx
dora --help
```

<details>
  <summary> For Macos </summary>

```bash
wget https://github.com/dora-rs/dora/releases/download/<version>/dora-<version>-x86_64-macOS.zip
unzip dora-<version>-x86_64-macOS.zip
PATH=$PATH:$(pwd):$(pwd)/iceoryx
dora --help
```

</details>

<details>
  <summary> For Windows </summary>

```bash
wget https://github.com/dora-rs/dora/releases/download/<version>/dora-<version>-x86_64-Windows.zip
unzip dora-<version>-x86_64-Windows.zip
PATH=$PATH:$(pwd):$(pwd)/iceoryx
dora --help
```

</details>

> This is `x86_64` only for the moment.

2. Create a new dataflow

```bash
dora new abc_project --lang python
cd abc_project
```

This creates the following `abc_project` directory
```bash
.
├── dataflow.yml
├── node_1
│   └── node_1.py
├── op_1
│   └── op_1.py
└── op_2
    └── op_2.py
```

3. Start `dora-coordinator` in a separate terminal window
```bash
# New terminal window
dora up 
```

4. Start your dataflow
```bash
# Other window
dora start dataflow.yml
# Output: c95d118b-cded-4531-a0e4-cd85b7c3916c
```
The output is the unique ID of the dataflow instance, which can be used to control it through the `dora` CLI.

5. You will see in your `dora-coordinator` window operators receiving ticks.
```bash
Received input tick, with data: b''
Received input tick, with data: b''
Received input tick, with data: b''
...
```

6. Stop your dataflow
```bash
dora stop c95d118b-cded-4531-a0e4-cd85b7c3916c
```
(Pass the ID returned by `dora start` here.)

7. You can then add or modify operators or nodes. For adding nodes easily, you can use the `dora` CLI again:

- Run `dora new --kind operator --lang Rust <name>` to create a new Rust operator named `<name>`.
- Run `dora new --kind custom-node --lang Rust <name>` to create a new custom node named `<name>`.

You need to add the created operators/nodes to your dataflow YAML file.

8. You can also download already implemented operators by putting links in the dataflow. This example will launch a webcam plot stream. 

```yaml
communication:
  zenoh:
    prefix: /abc_project

nodes:
  - id: op_1
    operator:
      python: https://raw.githubusercontent.com/dora-rs/dora-drives/main/operators/webcam.py
      inputs:
        tick: dora/timer/millis/100
      outputs:
        - image
  - id: op_2
    operator:
      python: https://raw.githubusercontent.com/dora-rs/dora-drives/main/physicals/plot.py
      inputs:
        image: op_1/image 
```
> Make sure to have a webcam and cv2 install via: `pip install opencv-python`
---

## ✨ Features

Composability as:
- [x] `YAML` declarative programming
- [x] language-agnostic:
  - [x] Rust
  - [x] C
  - [x] C++
  - [x] Python
- [x] Isolated operators and custom nodes that can be reused.

Low latency as:
- [x] written in  <i>...Cough...blazingly fast ...Cough...</i> Rust.
- [x] PubSub communication with [`iceoryx`](https://iceoryx.io/v1.0.1/)

Distributed as:
- [x] PubSub communication with [`zenoh`](https://github.com/eclipse-zenoh/zenoh)
- [x] Distributed telemetry with [`opentelemetry`](https://github.com/open-telemetry/opentelemetry-rust)

---


## 🏁 Further reading

- Check out [dora-drives](https://github.com/dora-rs/dora-drives) for a template of an autonomous vehicle within a simulation.


## ⚖️ LICENSE 

This project is licensed under Apache-2.0. Check out [NOTICE.md](NOTICE.md) for more information.
