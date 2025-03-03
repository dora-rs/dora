#

<p align="center">
    <img src="https://raw.githubusercontent.com/dora-rs/dora/main/docs/src/logo.svg" width="400"/>
</p>

<h2 align="center">
  <a href="https://www.dora-rs.ai">Website</a>
  |
  <a href="https://dora-rs.ai/docs/guides/getting-started/conversation_py/">Python API</a>
  |
  <a href="https://docs.rs/dora-node-api/latest/dora_node_api/">Rust API</a>
  |
  <a href="https://www.dora-rs.ai/docs/guides/">Guide</a>
  |
  <a href="https://discord.gg/6eMGGutkfE">Discord</a>
</h2>

<div align="center">
  <a href="https://github.com/dora-rs/dora/actions">
    <img src="https://github.com/dora-rs/dora/workflows/CI/badge.svg" alt="Build and test"/>
  </a>
  <a href="https://crates.io/crates/dora-rs">
    <img src="https://img.shields.io/crates/v/dora_node_api.svg"/>
  </a>
  <a href="https://docs.rs/dora-node-api/latest/dora_node_api/">
    <img src="https://docs.rs/dora-node-api/badge.svg" alt="rust docs"/>
  </a>
  <a href="https://pypi.org/project/dora-rs/">
    <img src="https://img.shields.io/pypi/v/dora-rs.svg" alt="PyPi Latest Release"/>
  </a>
</div>
<div align="center">
<a href="https://trendshift.io/repositories/9190" target="_blank"><img src="https://trendshift.io/api/badge/repositories/9190" alt="dora-rs%2Fdora | Trendshift" style="width: 250px; height: 55px;" width="250" height="55"/></a>
</div>

An extremely fast and simple **dataflow oriented robotic** framework to manage your projects and run complex **apps**, written in Rust.

<p align="center">
  <picture align="center">
    <source media="(prefers-color-scheme: dark)" srcset="https://raw.githubusercontent.com/dora-rs/dora/main/docs/src/bar_chart_dark.svg">
    <source media="(prefers-color-scheme: light)" srcset="https://raw.githubusercontent.com/dora-rs/dora/main/docs/src/bar_chart_light.svg">
    <img src="https://raw.githubusercontent.com/dora-rs/dora/main/docs/src/bar_chart_light.svg">
  </picture>
</p>

<p align="center">
 <a href="https://github.com/dora-rs/dora-benchmark/" >
  <i>Latency benchmark with Python API for both framework, sending 40M of random bytes.</i>
  </a>
</p>

## Latest News 🎉

<details open>
<summary><b>2025</b></summary>

- \[2025/03\] Add support for Meta SAM2, Kokoro(TTS), Improved Qwen2.5 Performance using `llama.cpp`.
- \[2025/02\] Add support for Qwen2.5(LLM), Qwen2.5-VL(VLM), outetts(TTS)

</details>

## Highlights

- 🚀 A single CLI to launch your Python and Rust robotic projects.
- ⚡️ [10-17x faster](https://github.com/dora-rs/dora-benchmark) than `ros2`.
- 🐍 Easy and Fast prototyping with a clean [Python API](https://dora-rs.ai/docs/guides/getting-started/conversation_py/).
- 🖥️ Supports macOS, Linux, and Windows.
- ⏬ Installable without Rust via `curl` or `powershell`.
- ❇️ Includes a large set of pre-packaged nodes for fast prototyping.
- 🛠️ Build and Run applications **without compilation step** beyond the native compiler of your favourite language.
- 🤖 Simplifies building robotic applications by integrating hardware, algorithms, and AI models to facilitate seamless communication.
- ⚙️ Eases integration of hardware and software by supporting Python, C, C++, and ROS2, while ensuring low-latency communication with zero-copy Arrow messages.

## Installation

Install dora with our standalone installers, or from [crates.io](https://crates.io/crates/dora-cli):

### With pip

```bash
pip install dora-rs-cli
```

### With cargo

```bash
cargo install dora-cli
```

### With Github release for macOS and Linux

```bash
curl --proto '=https' --tlsv1.2 -sSf https://raw.githubusercontent.com/dora-rs/dora/main/install.sh | bash
```

### With Github release for Windows

```powershell
powershell -c "irm https://raw.githubusercontent.com/dora-rs/dora/main/install.ps1 | iex"
```

## Documentation

The full documentation is available on [our website](https://dora-rs.ai/).
A lot of guides are available on [this section](https://dora-rs.ai/docs/guides/) of our website.

## Getting Started

1. Run some Python examples (A venv must be activated):

```bash
cd dora/examples/python-dataflow
uv venv --seed
dora build dataflow.yml --uv
dora run dataflow.yml --uv
```

> Make sure to have a webcam

To stop your dataflow, you can use <kbd>ctrl</kbd>+<kbd>c</kbd>

## What is Dora? And what features does Dora offer?

**D**ataflow-**O**riented **R**obotic **A**rchitecture (`dora-rs`) is a framework that makes creation of robotic applications fast and simple.

`dora-rs` implements a declarative dataflow paradigm where tasks are split between nodes isolated as individual processes.

Each node defines its inputs and outputs to connect with other nodes.

```yaml
nodes:
  - id: camera
    path: opencv-video-capture
    inputs:
      tick: dora/timer/millis/20
    outputs:
      - image
      -
  - id: plot
    path: opencv-plot
    inputs:
      image: camera/image
```

The dataflow paradigm has the advantage of creating an abstraction layer that makes robotic applications modular and easily configurable.

<p align="center">
  <picture align="center">
    <source media="(prefers-color-scheme: dark)" srcset="https://raw.githubusercontent.com/dora-rs/dora/main/docs/src/dora_diag_dark.svg">
    <source media="(prefers-color-scheme: light)" srcset="https://raw.githubusercontent.com/dora-rs/dora/main/docs/src/dora_diag_light.svg">
    <img src="https://raw.githubusercontent.com/dora-rs/dora/main/docs/src/dora_diag_light.svg">
  </picture>
</p>

---

It offers several features, such as:

### TCP Communication and Shared Memory

Communication between nodes is handled with shared memory on a same machine and TCP on distributed machines. Our shared memory implementation tracks messages across processes and discards them when obsolete. Shared memory slots are cached to avoid new memory allocation.

### Arrow Message Format

Nodes communicate with Apache Arrow Data Format.

[Apache Arrow](https://github.com/apache/arrow-rs) is a universal memory format for flat and hierarchical data. The Arrow memory format supports zero-copy reads for lightning-fast data access without serialization overhead. It defines a C data interface without any build-time or link-time dependency requirement, that means that `dora-rs` has **no compilation step** beyond the native compiler of your favourite language.

### Opentelemetry

dora-rs uses Opentelemetry to record all your logs, metrics and traces. This means that the data and telemetry can be linked using a shared abstraction.

[Opentelemetry](https://opentelemetry.io/) is an open source observability standard that makes dora-rs telemetry collectable by most backends such as elasticsearch, prometheus, Datadog...

Opentelemetry is language independent, backend agnostic, and easily collect distributed data, making it perfect for dora-rs applications.

### Hot-Reloading

dora-rs implements Hot-Reloading for python which means you can change code at runtime in Python while keeping your state intact.

Using the feature flag: `--attach --hot-reload`, dora-rs watch for code change and reload nodes that has been modified.

You can check fail-safe mechanism at: https://github.com/dora-rs/dora/pull/239.

See [this demo](http://www.youtube.com/watch?v=NvvTEP8Jak8).

### ROS2 Bridge

**Note**: this feature is marked as unstable.

- Compilation Free Message passing to ROS 2
- Automatic conversion ROS 2 Message <-> Arrow Array

```python
import pyarrow as pa

# Configuration Boilerplate...
turtle_twist_writer = ...

## Arrow Based ROS2 Twist Message
## which does not require ROS2 import
message = pa.array([{
            "linear": {
                "x": 1,
            },
            "angular": {
                "z": 1
            },
        }])

turtle_twist_writer.publish(message)
```

> You might want to use ChatGPT to write the Arrow Formatting: https://chat.openai.com/share/4eec1c6d-dbd2-46dc-b6cd-310d2895ba15

## Showcases

### Self-Coding Robot: Code RAG (WIP)

You can easily create a self-coding robot, by combining Hot-reloading with a Retrieval Augmented Generation (RAG) that is going to generate code modification from your prompt.
See:[examples/python-operator-dataflow](examples/python-operator-dataflow)

<p align="center">
  <picture align="center">
    <source media="(prefers-color-scheme: dark)" srcset="https://raw.githubusercontent.com/dora-rs/dora/main/docs/src/self_coding_dark.svg">
    <source media="(prefers-color-scheme: light)" srcset="https://raw.githubusercontent.com/dora-rs/dora/main/docs/src/self_coding_light.svg">
    <img src="https://raw.githubusercontent.com/dora-rs/dora/main/docs/src/self_coding_light.svg">
  </picture>
</p>

Self-Coding Robot is just the tip of the iceberg of robotics combined with llm, that we hope to power. There is so much more that we haven't explored yet like:

- [self-debugging](https://arxiv.org/pdf/2304.05128.pdf)
- [memory](https://github.com/cpacker/MemGPT)
- [function calling](https://github.com/ShishirPatil/gorilla)

### Cool Hardwares

Cool hardware that we think might be good fit to try out dora-rs 🙋 We are not sponsored by manufacturers:

|                                   | Price | Open Source        | Github                                               | type       | Dora Project                                            |
| --------------------------------- | ----- | ------------------ | ---------------------------------------------------- | ---------- | ------------------------------------------------------- |
| DJI Robomaster S1                 | 550$  | SDK                | https://github.com/dji-sdk/RoboMaster-SDK            | Rover      | https://huggingface.co/datasets/dora-rs/dora-robomaster |
| DJI Robomaster EP Core            | 950$  | SDK                | https://github.com/dji-sdk/RoboMaster-SDK            | Rover, Arm |                                                         |
| DJI Tello                         | 100$  |                    |                                                      | Drone      |                                                         |
| BitCraze Crazyflies               | 225$  | Firmware, Lib, SDK | https://github.com/bitcraze                          | Drone      |                                                         |
| AlexanderKoch-Koch/low_cost_robot | 250$  | Everything         | https://github.com/AlexanderKoch-Koch/low_cost_robot | Arm        |                                                         |
| xArm 1S                           | 200$  |                    |                                                      | Arm        |                                                         |
| Wavego                            | 250$  |                    |                                                      | Quadruplet |                                                         |
| AINex                             | 800$  |                    |                                                      | Humanoid   |                                                         |

> For more: https://docs.google.com/spreadsheets/d/1YYeW2jfOIWDVgdEgqnMvltonHquQ7K8OZCrnJRELL6o/edit#gid=0

## Support Matrix

|                                   | dora-rs                                                                                 | Hoped for                                                                                                                                |
| --------------------------------- | --------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------- |
| **Tier 1 Support**                | Python, Rust                                                                            | C, C++, ROS 2                                                                                                                            |
| **Tier 2 Support**                | C, C++, ROS2                                                                            |
| **Hot-reloading**                 | Python                                                                                  | Rust (https://github.com/orgs/dora-rs/discussions/360)                                                                                   |
| **Message Format**                | Arrow                                                                                   | Native                                                                                                                                   |
| **Local Communication**           | Shared Memory                                                                           | Custom Middleware, [zero-copy GPU IPC](https://arrow.apache.org/docs/python/api/cuda.html), intra-process `tokio::channel` communication |
| **Remote Communication**          | TCP                                                                                     | Custom Middleware, [Zenoh](https://zenoh.io/)                                                                                            |
| **Metrics, Tracing, and Logging** | Opentelemetry                                                                           | Native logging libraries into Opentelemetry                                                                                              |
| **Data archives**                 | Parquet ([dora-record](https://github.com/dora-rs/dora/tree/main/node-hub/dora-record)) |
| **Visualization and annotation**  | OpenCV                                                                                  | [rerun.io](rerun.io)                                                                                                                     |
| **Supported Platforms (x86)**     | Windows, macOS, Linux                                                                   |
| **Supported Platforms (ARM)**     | macOS, Linux                                                                            |
| **Configuration**                 | YAML                                                                                    |

## Contributing

We are passionate about supporting contributors of all levels of experience and would love to see
you get involved in the project. See the
[contributing guide](https://github.com/dora-rs/dora/blob/main/CONTRIBUTING.md) to get started.

## Discussions

Our main communication channels are:

- [Our Discord server](https://discord.gg/6eMGGutkfE)
- [Our Github Project Discussion](https://github.com/orgs/dora-rs/discussions)

Feel free to reach out on any topic, issues or ideas.

We also have [a contributing guide](CONTRIBUTING.md).

## License

This project is licensed under Apache-2.0. Check out [NOTICE.md](NOTICE.md) for more information.
