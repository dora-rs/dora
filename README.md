#

<p align="center">
    <img src="https://raw.githubusercontent.com/adora-rs/adora/main/docs/src/logo.svg" width="400"/>
</p>

<h2 align="center">
  <a href="https://www.adora-rs.ai">Website</a>
  |
  <a href="https://adora-rs.ai/docs/guides/getting-started/conversation_py/">Python API</a>
  |
  <a href="https://docs.rs/adora-node-api/latest/adora_node_api/">Rust API</a>
  |
  <a href="https://www.adora-rs.ai/docs/guides/">Guide</a>
  |
  <a href="https://discord.gg/6eMGGutkfE">Discord</a>
</h2>

<div align="center">
  <a href="https://github.com/adora-rs/adora/actions">
    <img src="https://github.com/adora-rs/adora/workflows/CI/badge.svg" alt="Build and test"/>
  </a>
  <a href="https://crates.io/crates/adora-rs">
    <img src="https://img.shields.io/crates/v/adora_node_api.svg"/>
  </a>
  <a href="https://docs.rs/adora-node-api/latest/adora_node_api/">
    <img src="https://docs.rs/adora-node-api/badge.svg" alt="rust docs"/>
  </a>
  <a href="https://pypi.org/project/adora-rs/">
    <img src="https://img.shields.io/pypi/v/adora-rs.svg" alt="PyPi Latest Release"/>
  </a>
    <a href="https://github.com/adora-rs/adora/blob/main/LICENSE">
    <img src="https://img.shields.io/github/license/adora-rs/adora" alt="PyPi Latest Release"/>
  </a>      
</div>
<div align="center">
<a href="https://trendshift.io/repositories/9190" target="_blank"><img src="https://trendshift.io/api/badge/repositories/9190" alt="adora-rs%2Fadora | Trendshift" style="width: 250px; height: 55px;" width="250" height="55"/></a>
</div>

<div align="center">
  <b>This repository is maintained with AI-assisted agentic engineering using <a href="https://claude.ai/code">Claude Code</a>.</b><br/>
  Code reviews, refactoring, testing, and commits are driven by autonomous AI agents — enabling faster iteration and higher code quality at scale.
</div>

## Highlights

- 🚀 adora-rs is a framework to run realtime multi-AI and multi-hardware applications.
- 🦀 adora-rs internals are 100% Rust making it extremely fast compared to alternative such as being ⚡️ [10-17x faster](https://github.com/adora-rs/adora-benchmark) than `ros2`.
- ❇️ Includes a large set of pre-packaged nodes for fast prototyping which simplifies integration of hardware, algorithms, and AI models.

<p align="center">
  <picture align="center">
    <source media="(prefers-color-scheme: dark)" srcset="https://raw.githubusercontent.com/adora-rs/adora/main/docs/src/bar_chart_dark.svg">
    <source media="(prefers-color-scheme: light)" srcset="https://raw.githubusercontent.com/adora-rs/adora/main/docs/src/bar_chart_light.svg">
    <img src="https://raw.githubusercontent.com/adora-rs/adora/main/docs/src/bar_chart_light.svg">
  </picture>
</p>

<p align="center">
 <a href="https://github.com/adora-rs/adora-benchmark/" >
  <i>Latency benchmark with Python API for both framework, sending 40M of random bytes.</i>
  </a>
</p>

## Latest News 🎉

<details open>
<summary><b>2025</b></summary>

- \[08/25\] Introduced `adora.builder`, a new Pythonic API for imperatively defining `adora` dataflows.
- \[07/25\] Added Kornia rust nodes in the hub for V4L / Gstreamer cameras and Sobel image processing.
- \[06/25\] Add support for git based node, adora-vggt for multi-camera depth estimation, and adding robot_descriptions_py as a default way to get urdfs within adora.
- \[05/25\] Add support for adora-pytorch-kinematics for fk and ik, adora-mediapipe for pose estimation, adora-rustypot for rust serialport read/write, points2d and points3d visualization in rerun.
- \[04/25\] Add support for adora-cotracker to track any point on a frame, adora-rav1e AV1 encoding up to 12bit and adora-dav1d AV1 decoding,
- \[03/25\] Add support for adora async Python.
- \[03/25\] Add support for Microsoft Phi4, Microsoft Magma.
- \[03/25\] adora-rs has been accepted to [**GSoC 2025 🎉**](https://summerofcode.withgoogle.com/programs/2025/organizations/adora-rs-tb), with the following [**idea list**](https://github.com/adora-rs/adora/wiki/GSoC_2025).
- \[03/25\] Add support for Zenoh for distributed dataflow.
- \[03/25\] Add support for Meta SAM2, Kokoro(TTS), Improved Qwen2.5 Performance using `llama.cpp`.
- \[02/25\] Add support for Qwen2.5(LLM), Qwen2.5-VL(VLM), outetts(TTS)
</details>

## Support Matrix

|                                   | adora-rs                                                                                                                                                                                                     |
| --------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **APIs**                          | Python >= 3.8 including sync ⭐✅ <br> Rust ✅<br> C/C++ 🆗 <br>ROS2 >= Foxy 🆗                                                                                                                             |
| **OS**                            | Linux: Arm 32 ⭐✅ Arm 64 ⭐✅ x64_86 ⭐✅ <br>MacOS: Arm 64 ⭐✅ <br>Windows: x64_86 🆗 <br>WSL: x64_86 🆗 <br> Android: 🛠️ (Blocked by: https://github.com/elast0ny/shared_memory/issues/32) <br> IOS: 🛠️ |
| **Message Format**                | Arrow ✅ <br> Standard Specification 🛠️                                                                                                                                                                     |
| **Local Communication**           | Shared Memory ✅ <br> [Cuda IPC](https://arrow.apache.org/docs/python/api/cuda.html) 📐                                                                                                                     |
| **Remote Communication**          | [Zenoh](https://zenoh.io/) 📐                                                                                                                                                                               |
| **RGB-D Streaming**               | AV1 Encoding (adora-rav1e), AV1 Decoding (adora-dav1d)📐                                                                                                                                                      |
| **Metrics, Tracing, and Logging** | Opentelemetry 📐                                                                                                                                                                                            |
| **Configuration**                 | YAML ✅                                                                                                                                                                                                     |
| **Package Manager**               | [pip](https://pypi.org/): Python Node ✅ Rust Node ✅ C/C++ Node 🛠️ <br>[cargo](https://crates.io/): Rust Node ✅                                                                                           |

> - ⭐ = Recommended
> - ✅ = First Class Support
> - 🆗 = Best Effort Support
> - 📐 = Experimental and looking for contributions
> - 🛠️ = Unsupported but hoped for through contributions
>
> Everything is open for contributions 🙋

## Node Hub

The node hub is available in the [**`dora-rs/dora-hub`**](https://github.com/dora-rs/dora-hub/) repository.

## Examples

| Type      | Title                                                                                                          | Description                             | Last Commit                                                                                                          |
| --------- | -------------------------------------------------------------------------------------------------------------- | --------------------------------------- | -------------------------------------------------------------------------------------------------------------------- |
| Vision    | [YOLO](https://github.com/adora-rs/adora/blob/main/examples/python-dataflow)                                     | Use YOLO to detect object within image. | ![License](https://img.shields.io/github/last-commit/adora-rs/adora?path=examples%2Fpython-dataflow&label=%20)         |
| ROS2      | [C++ ROS2 Example](https://github.com/adora-rs/adora/blob/main/examples/c++-ros2-dataflow)                       | Example using C++ ROS2                  | ![License](https://img.shields.io/github/last-commit/adora-rs/adora?path=examples%2Fc%2b%2b-ros2-dataflow&label=%20)   |
| ROS2      | [Rust ROS2 Example](https://github.com/adora-rs/adora/blob/main/examples/rust-ros2-dataflow)                     | Example using Rust ROS2                 | ![License](https://img.shields.io/github/last-commit/adora-rs/adora?path=examples%2Frust-ros2-dataflow&label=%20)      |
| ROS2      | [Python ROS2 Example](https://github.com/adora-rs/adora/blob/main/examples/python-ros2-dataflow)                 | Example using Python ROS2               | ![License](https://img.shields.io/github/last-commit/adora-rs/adora?path=examples%2Fpython-ros2-dataflow&label=%20)    |
| Benchmark | [GPU Benchmark](https://github.com/adora-rs/adora/blob/main/examples/cuda-benchmark)                             | GPU Benchmark of adora-rs                | ![License](https://img.shields.io/github/last-commit/adora-rs/adora?path=examples%2Fcuda-benchmark&label=%20)          |
| Benchmark | [CPU Benchmark](https://github.com/adora-rs/adora-benchmark/blob/main)                                           | CPU Benchmark of adora-rs                | ![License](https://img.shields.io/github/last-commit/adora-rs/adora-benchmark?path=adora-rs&label=%20)                  |
| Tutorial  | [Rust Example](https://github.com/adora-rs/adora/blob/main/examples/rust-dataflow)                               | Example using Rust                      | ![License](https://img.shields.io/github/last-commit/adora-rs/adora?path=examples%2Frust-dataflow&label=%20)           |
| Tutorial  | [Python Example](https://github.com/adora-rs/adora/blob/main/examples/python-dataflow)                           | Example using Python                    | ![License](https://img.shields.io/github/last-commit/adora-rs/adora?path=examples%2Fpython-dataflow&label=%20)         |
| Tutorial  | [CMake Example](https://github.com/adora-rs/adora/blob/main/examples/cmake-dataflow)                             | Example using CMake                     | ![License](https://img.shields.io/github/last-commit/adora-rs/adora?path=examples%2Fcmake-dataflow&label=%20)          |
| Tutorial  | [C Example](https://github.com/adora-rs/adora/blob/main/examples/c-dataflow)                                     | Example with C node                     | ![License](https://img.shields.io/github/last-commit/adora-rs/adora?path=examples%2Fc-dataflow&label=%20)              |
| Tutorial  | [CUDA Example](https://github.com/adora-rs/adora/blob/main/examples/cuda-benchmark)                              | Example using CUDA Zero Copy            | ![License](https://img.shields.io/github/last-commit/adora-rs/adora?path=examples%2Fcuda-benchmark&label=%20)          |
| Tutorial  | [C++ Example](https://github.com/adora-rs/adora/blob/main/examples/c++-dataflow)                                 | Example with C++ node                   | ![License](https://img.shields.io/github/last-commit/adora-rs/adora?path=examples%2Fc%2b%2b-dataflow&label=%20)        |
| Tutorial  | [Python Dataflow Builder Examples](https://github.com/adora-rs/adora/blob/main/examples/python-dataflow-builder) | Examples using the new Pythonic API.    | ![License](https://img.shields.io/github/last-commit/adora-rs/adora?path=examples%2Fpython-dataflow-builder&label=%20) |

=

## Getting Started

### Installation

```bash
pip install adora-rs-cli
```

<details close>
<summary><b>Additional installation methods</b></summary>

Install adora with our standalone installers, or from [crates.io](https://crates.io/crates/adora-cli):

### With cargo

```bash
cargo install adora-cli
```

### With Github release for macOS and Linux

```bash
curl --proto '=https' --tlsv1.2 -LsSf https://github.com/adora-rs/adora/releases/latest/download/adora-cli-installer.sh | sh
```

To install a specific version:

```bash
curl --proto '=https' --tlsv1.2 -LsSf https://raw.githubusercontent.com/adora-rs/adora/main/install.sh | sh -s -- --tag v0.3.12
```

### With Github release for Windows

```powershell
powershell -ExecutionPolicy ByPass -c "irm https://github.com/adora-rs/adora/releases/latest/download/adora-cli-installer.ps1 | iex"
```

### With Source

```bash
git clone https://github.com/adora-rs/adora.git
cd adora
cargo build --release -p adora-cli
PATH=$PATH:$(pwd)/target/release
```

</details>

### Run

- Run the yolo python example:

```bash
## Create a virtual environment
uv venv --seed -p 3.11

## Install nodes dependencies of a remote graph
adora build https://raw.githubusercontent.com/adora-rs/adora/refs/heads/main/examples/python-dataflow/dataflow.yml --uv

## Run yolo graph
adora run dataflow.yml --uv
```
> Make sure to have a webcam

To stop your dataflow, you can use <kbd>ctrl</kbd>+<kbd>c</kbd>

- To understand what is happening, you can look at the dataflow with:

```bash
cat dataflow.yml
```

- Resulting in:

```yaml
nodes:
  - id: camera
    build: pip install opencv-video-capture
    path: opencv-video-capture
    inputs:
      tick: adora/timer/millis/20
    outputs:
      - image
    env:
      CAPTURE_PATH: 0
      IMAGE_WIDTH: 640
      IMAGE_HEIGHT: 480

  - id: object-detection
    build: pip install adora-yolo
    path: adora-yolo
    inputs:
      image: camera/image
    outputs:
      - bbox

  - id: plot
    build: pip install adora-rerun
    path: adora-rerun
    inputs:
      image: camera/image
      boxes2d: object-detection/bbox
```

- In the above example, we can understand that the camera is sending image to both the rerun viewer as well as a yolo model that generates bounding box that is visualized within rerun.

### Documentation

The full documentation is available on [our website](https://adora-rs.ai/).
A lot of guides are available on [this section](https://adora-rs.ai/docs/guides/) of our website.

## What is Adora? And what features does Adora offer?

**D**ataflow-**O**riented **R**obotic **A**rchitecture (`adora-rs`) is a framework that makes creation of robotic applications fast and simple.

`adora-rs` implements a declarative dataflow paradigm where tasks are split between nodes isolated as individual processes.

The dataflow paradigm has the advantage of creating an abstraction layer that makes robotic applications modular and easily configurable.

### TCP Communication and Shared Memory

Communication between nodes is handled with shared memory on a same machine and TCP on distributed machines. Our shared memory implementation tracks messages across processes and discards them when obsolete. Shared memory slots are cached to avoid new memory allocation.

### Arrow Message Format

Nodes communicate with Apache Arrow Data Format.

[Apache Arrow](https://github.com/apache/arrow-rs) is a universal memory format for flat and hierarchical data. The Arrow memory format supports zero-copy reads for lightning-fast data access without serialization overhead. It defines a C data interface without any build-time or link-time dependency requirement, that means that `adora-rs` has **no compilation step** beyond the native compiler of your favourite language.

### Opentelemetry

adora-rs uses Opentelemetry to record all your logs, metrics and traces. This means that the data and telemetry can be linked using a shared abstraction.

[Opentelemetry](https://opentelemetry.io/) is an open source observability standard that makes adora-rs telemetry collectable by most backends such as elasticsearch, prometheus, Datadog...

Opentelemetry is language independent, backend agnostic, and easily collect distributed data, making it perfect for adora-rs applications.

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

## Contributing

We are passionate about supporting contributors of all levels of experience and would love to see
you get involved in the project. See the
[contributing guide](https://github.com/adora-rs/adora/blob/main/CONTRIBUTING.md) to get started.

## Discussions

Our main communication channels are:

- [Our Discord server](https://discord.gg/6eMGGutkfE)
- [Our Github Project Discussion](https://github.com/orgs/adora-rs/discussions)

Feel free to reach out on any topic, issues or ideas.

We also have [a contributing guide](CONTRIBUTING.md).

## License

This project is licensed under Apache-2.0. Check out [NOTICE.md](NOTICE.md) for more information.

---

## Further Resources 📚

- [Zenoh Documentation](https://zenoh.io/docs/getting-started/first-app/)
- [ADORA Zenoh Discussion (GitHub Issue #512)](https://github.com/adora-rs/adora/issues/512)
- [Adora Autoware Localization Demo](https://github.com/adora-rs/adora-autoware-localization-demo)

```

```
