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

An extremely fast and simple **dataflow oriented robotic** framework to manage your projects and run realtime multi-AI and multi-hardware **applications**, written in Rust.

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

- \[2025/03/05\] dora-rs has been accepted to [**GSoC 2025 🎉**](https://summerofcode.withgoogle.com/programs/2025/organizations/dora-rs-tb), with the following [**idea list**](https://github.com/dora-rs/dora/wiki/GSoC_2025).
- \[2025/03/04\] Add support for Zenoh for distributed dataflow.
- \[2025/03/04\] Add support for Meta SAM2, Kokoro(TTS), Improved Qwen2.5 Performance using `llama.cpp`.
- \[2025/02/25\] Add support for Qwen2.5(LLM), Qwen2.5-VL(VLM), outetts(TTS)
</details>

## Highlights

- 🚀 A single CLI to run multiple AI models and hardware drivers in parallel.
- 🦀 dora-rs is 100% Rust project.
- 🖥️ Supports macOS, Linux, and Windows.
- ⚙️ Support Python, C, C++, and ROS2, while ensuring low-latency communication with zero-copy Arrow messages.
- ⏬ Everything is `pip`, `cargo`, or `curl` installable.
- ❇️ Includes a large set of pre-packaged nodes for fast prototyping.
- 🛠️ Build and Run applications **without compilation step** beyond the native compiler of your favourite language.
- 🤖 Simplifies building robotic applications by integrating hardware, algorithms, and AI models to facilitate seamless communication.
- ⚡️ [10-17x faster](https://github.com/dora-rs/dora-benchmark) than `ros2`.

## Node Hub

### Camera

| Title                                                                                    | Description                         | Downloads                                                         | License                                                        | Release                                                        |
| ---------------------------------------------------------------------------------------- | ----------------------------------- | ----------------------------------------------------------------- | -------------------------------------------------------------- | -------------------------------------------------------------- |
| [PyOrbbeckSDK](https://github.com/dora-rs/dora/blob/main/node-hub/dora-pyorbbecksdk)     | Image and depth from Orbbeck Camera | ![Downloads](https://img.shields.io/pypi/dm/dora-pyorbbecksdk)    | ![License](https://img.shields.io/pypi/l/dora-pyorbbecksdk)    | ![Release](https://img.shields.io/pypi/v/dora-pyorbbecksdk)    |
| [PyRealsense](https://github.com/dora-rs/dora/blob/main/node-hub/dora-pyrealsense)       | Image and depth from Realsense      | ![Downloads](https://img.shields.io/pypi/dm/dora-pyrealsense)     | ![License](https://img.shields.io/pypi/l/dora-pyrealsense)     | ![Release](https://img.shields.io/pypi/v/dora-pyrealsense)     |
| [Video Capture](https://github.com/dora-rs/dora/blob/main/node-hub/opencv-video-capture) | Image stream from Camera            | ![Downloads](https://img.shields.io/pypi/dm/opencv-video-capture) | ![License](https://img.shields.io/pypi/l/opencv-video-capture) | ![Release](https://img.shields.io/pypi/v/opencv-video-capture) |

### Peripheral

| Title                                                                               | Description               | Downloads                                                    | License                                                   | Release                                                   |
| ----------------------------------------------------------------------------------- | ------------------------- | ------------------------------------------------------------ | --------------------------------------------------------- | --------------------------------------------------------- |
| [PyAudio(Speaker)](https://github.com/dora-rs/dora/blob/main/node-hub/dora-pyaudio) | Output audio from speaker | ![Downloads](https://img.shields.io/pypi/dm/dora-pyaudio)    | ![License](https://img.shields.io/pypi/l/dora-pyaudio)    | ![Release](https://img.shields.io/pypi/v/dora-pyaudio)    |
| [Microphone](https://github.com/dora-rs/dora/blob/main/node-hub/dora-microphone)    | Audio from microphone     | ![Downloads](https://img.shields.io/pypi/dm/dora-microphone) | ![License](https://img.shields.io/pypi/l/dora-microphone) | ![Release](https://img.shields.io/pypi/v/dora-microphone) |
| [Keyboard](https://github.com/dora-rs/dora/blob/main/node-hub/dora-keyboard)        | Keyboard char listener    | ![Downloads](https://img.shields.io/pypi/dm/dora-keyboard)   | ![License](https://img.shields.io/pypi/l/dora-keyboard)   | ![Release](https://img.shields.io/pypi/v/dora-keyboard)   |

### Actuator

| Title                                                                                    | Description      | Downloads | License | Release |
| ---------------------------------------------------------------------------------------- | ---------------- | --------- | ------- | ------- |
| [Feetech](https://github.com/dora-rs/dora-lerobot/blob/main/node-hub/feetech-client)     | Feetech Client   |           |         |         |
| [Dynamixel](https://github.com/dora-rs/dora-lerobot/blob/main/node-hub/dynamixel-client) | Dynamixel Client |           |         |         |

### Chassis

| Title                                                                           | Description         | Downloads                                                 | License                                                | Release                                                |
| ------------------------------------------------------------------------------- | ------------------- | --------------------------------------------------------- | ------------------------------------------------------ | ------------------------------------------------------ |
| [Dora Kit Car](https://github.com/dora-rs/dora/blob/main/node-hub/dora-kit-car) | Open Source Chassis | ![Downloads](https://img.shields.io/pypi/dm/dora-kit-car) | ![License](https://img.shields.io/pypi/l/dora-kit-car) | ![Release](https://img.shields.io/pypi/v/dora-kit-car) |
| [Agilex - UGV](https://github.com/dora-rs/dora/blob/main/node-hub/dora-ugv)     | Robomaster Client   | ![Downloads](https://img.shields.io/pypi/dm/dora-ugv)     | ![License](https://img.shields.io/pypi/l/dora-ugv)     | ![Release](https://img.shields.io/pypi/v/dora-ugv)     |
| [DJI - Robomaster S1](https://huggingface.co/datasets/dora-rs/dora-robomaster)  | Robomaster Client   |                                                           |                                                        |                                                        |

### Arm

| Title                                                                                            | Description                       | Downloads                                               | License                                              | Release                                              |
| ------------------------------------------------------------------------------------------------ | --------------------------------- | ------------------------------------------------------- | ---------------------------------------------------- | ---------------------------------------------------- |
| [Alex Koch - Low Cost Robot](https://github.com/dora-rs/dora-lerobot/blob/main/robots/alexk-lcr) | Alex Koch - Low Cost Robot Client |                                                         |                                                      |                                                      |
| [Lebai - LM3](https://github.com/dora-rs/dora-lerobot/blob/main/node-hub/lebai-client)           | Lebai client                      |                                                         |                                                      |                                                      |
| [Agilex - Piper](https://github.com/dora-rs/dora/blob/main/node-hub/dora-piper)                  | Agilex arm client                 | ![Downloads](https://img.shields.io/pypi/dm/dora-piper) | ![License](https://img.shields.io/pypi/l/dora-piper) | ![Release](https://img.shields.io/pypi/v/dora-piper) |

### Robot

| Title                                                                                        | Description     | Downloads                                                 | License                                                | Release                                                |
| -------------------------------------------------------------------------------------------- | --------------- | --------------------------------------------------------- | ------------------------------------------------------ | ------------------------------------------------------ |
| [Pollen - Reachy 2](https://github.com/dora-rs/dora/blob/main/node-hub/dora-reachy2)         | Reachy 2 client | ![Downloads](https://img.shields.io/pypi/dm/dora-reachy2) | ![License](https://img.shields.io/pypi/l/dora-reachy2) | ![Release](https://img.shields.io/pypi/v/dora-reachy2) |
| [Trossen - Aloha](https://github.com/dora-rs/dora-lerobot/blob/main/robots/aloha)            | Aloha client    |                                                           |                                                        |                                                        |
| [Pollen - Reachy 1](https://github.com/dora-rs/dora-lerobot/blob/main/node-hub/dora-reachy1) | Reachy 1 Client |                                                           |                                                        |                                                        |

### Voice Activity Detection

| Title                                                                     | Description                     | Downloads                                             | License                                            | Release                                            |
| ------------------------------------------------------------------------- | ------------------------------- | ----------------------------------------------------- | -------------------------------------------------- | -------------------------------------------------- |
| [Silero VAD](https://github.com/dora-rs/dora/blob/main/node-hub/dora-vad) | Silero Voice activity detection | ![Downloads](https://img.shields.io/pypi/dm/dora-vad) | ![License](https://img.shields.io/pypi/l/dora-vad) | ![Release](https://img.shields.io/pypi/v/dora-vad) |

### Speech to Text

| Title                                                                             | Description              | Downloads                                                        | License                                                       | Release                                                       |
| --------------------------------------------------------------------------------- | ------------------------ | ---------------------------------------------------------------- | ------------------------------------------------------------- | ------------------------------------------------------------- |
| [Whisper](https://github.com/dora-rs/dora/blob/main/node-hub/dora-distil-whisper) | Transcribe audio to text | ![Downloads](https://img.shields.io/pypi/dm/dora-distil-whisper) | ![License](https://img.shields.io/pypi/l/dora-distil-whisper) | ![Release](https://img.shields.io/pypi/v/dora-distil-whisper) |

### Object Detection

| Title                                                                  | Description      | Downloads                                              | License                                             | Release                                             |
| ---------------------------------------------------------------------- | ---------------- | ------------------------------------------------------ | --------------------------------------------------- | --------------------------------------------------- |
| [Yolov8](https://github.com/dora-rs/dora/blob/main/node-hub/dora-yolo) | Object detection | ![Downloads](https://img.shields.io/pypi/dm/dora-yolo) | ![License](https://img.shields.io/pypi/l/dora-yolo) | ![Release](https://img.shields.io/pypi/v/dora-yolo) |

### Vision Language Model

| Title                                                                            | Description                            | Downloads                                                    | License                                                   | Release                                                   |
| -------------------------------------------------------------------------------- | -------------------------------------- | ------------------------------------------------------------ | --------------------------------------------------------- | --------------------------------------------------------- |
| [Qwen2.5-vl](https://github.com/dora-rs/dora/blob/main/node-hub/dora-qwen2-5-vl) | Vision Language Model using Qwen2.5 VL | ![Downloads](https://img.shields.io/pypi/dm/dora-qwen2-5-vl) | ![License](https://img.shields.io/pypi/l/dora-qwen2-5-vl) | ![Release](https://img.shields.io/pypi/v/dora-qwen2-5-vl) |
| [InternVL](https://github.com/dora-rs/dora/blob/main/node-hub/dora-internvl)     | InternVL is a vision language model    | ![Downloads](https://img.shields.io/pypi/dm/dora-internvl)   | ![License](https://img.shields.io/pypi/l/dora-internvl)   | ![Release](https://img.shields.io/pypi/v/dora-internvl)   |

### Large Language Model

| Title                                                                   | Description                     | Downloads                                              | License                                             | Release                                             |
| ----------------------------------------------------------------------- | ------------------------------- | ------------------------------------------------------ | --------------------------------------------------- | --------------------------------------------------- |
| [Qwen2.5](https://github.com/dora-rs/dora/blob/main/node-hub/dora-qwen) | Large Language Model using Qwen | ![Downloads](https://img.shields.io/pypi/dm/dora-qwen) | ![License](https://img.shields.io/pypi/l/dora-qwen) | ![Release](https://img.shields.io/pypi/v/dora-qwen) |

### Vision Language Action

| Title                                                                    | Description                                      | Downloads                                                | License                                               | Release                                               |
| ------------------------------------------------------------------------ | ------------------------------------------------ | -------------------------------------------------------- | ----------------------------------------------------- | ----------------------------------------------------- |
| [RDT-1B](https://github.com/dora-rs/dora/blob/main/node-hub/dora-rdt-1b) | Infer policy using Robotic Diffusion Transformer | ![Downloads](https://img.shields.io/pypi/dm/dora-rdt-1b) | ![License](https://img.shields.io/pypi/l/dora-rdt-1b) | ![Release](https://img.shields.io/pypi/v/dora-rdt-1b) |

### Translation

| Title                                                                                   | Description                     | Downloads                                                       | License                                                      | Release                                                      |
| --------------------------------------------------------------------------------------- | ------------------------------- | --------------------------------------------------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| [Opus MT](https://github.com/dora-rs/dora/blob/main/node-hub/dora-opus)                 | Translate text between language | ![Downloads](https://img.shields.io/pypi/dm/dora-opus)          | ![License](https://img.shields.io/pypi/l/dora-opus)          | ![Release](https://img.shields.io/pypi/v/dora-opus)          |
| [ArgosTranslate](https://github.com/dora-rs/dora/blob/main/node-hub/dora-argotranslate) | Open Source translation engine  | ![Downloads](https://img.shields.io/pypi/dm/dora-argotranslate) | ![License](https://img.shields.io/pypi/l/dora-argotranslate) | ![Release](https://img.shields.io/pypi/v/dora-argotranslate) |

### Text to Speech

| Title                                                                            | Description              | Downloads                                                    | License                                                   | Release                                                   |
| -------------------------------------------------------------------------------- | ------------------------ | ------------------------------------------------------------ | --------------------------------------------------------- | --------------------------------------------------------- |
| [Kokoro TTS](https://github.com/dora-rs/dora/blob/main/node-hub/dora-kokoro-tts) | Efficient Text to Speech | ![Downloads](https://img.shields.io/pypi/dm/dora-kokoro-tts) | ![License](https://img.shields.io/pypi/l/dora-kokoro-tts) | ![Release](https://img.shields.io/pypi/v/dora-kokoro-tts) |

### Recorder

| Title                                                                                               | Description                      | Downloads                                                           | License                                                          | Release                                                          |
| --------------------------------------------------------------------------------------------------- | -------------------------------- | ------------------------------------------------------------------- | ---------------------------------------------------------------- | ---------------------------------------------------------------- |
| [Llama Factory Recorder](https://github.com/dora-rs/dora/blob/main/node-hub/llama-factory-recorder) | Record data to train LLM and VLM | ![Downloads](https://img.shields.io/pypi/dm/llama-factory-recorder) | ![License](https://img.shields.io/pypi/l/llama-factory-recorder) | ![Release](https://img.shields.io/pypi/v/llama-factory-recorder) |
| [LeRobot Recorder](https://github.com/dora-rs/dora-lerobot/blob/main/node-hub/lerobot-dashboard)    | LeRobot Recorder helper          |                                                                     |                                                                  |                                                                  |

### Visualization

| Title                                                                  | Description                      | Downloads                                               | License                                               | Release                                               |
| ---------------------------------------------------------------------- | -------------------------------- | ------------------------------------------------------- | ----------------------------------------------------- | ----------------------------------------------------- |
| [Plot](https://github.com/dora-rs/dora/blob/main/node-hub/opencv-plot) | Simple OpenCV plot visualization | ![Downloads](https://img.shields.io/pypi/dm/dora-yolo)  | ![License](https://img.shields.io/pypi/l/opencv-plot) | ![Release](https://img.shields.io/pypi/v/opencv-plot) |
| [Rerun](https://github.com/dora-rs/dora/blob/main/node-hub/dora-rerun) | Visualization tool               | ![Downloads](https://img.shields.io/pypi/dm/dora-rerun) | ![License](https://img.shields.io/pypi/l/dora-rerun)  | ![Release](https://img.shields.io/pypi/v/dora-rerun)  |

### Simulator

| Title                                                                              | Description                          | Downloads | License | Release |
| ---------------------------------------------------------------------------------- | ------------------------------------ | --------- | ------- | ------- |
| [Mujoco](https://github.com/dora-rs/dora-lerobot/blob/main/node-hub/mujoco-client) | Mujoco Simulator                     |           |         |         |
| [Carla](https://github.com/dora-rs/dora-drives)                                    | Carla Simulator                      |           |         |         |
| [Gymnasium](https://github.com/dora-rs/dora-lerobot/blob/main/gym_dora)            | Experimental OpenAI Gymnasium bridge |           |         |         |

## Getting Started

### Installation

```bash
pip install dora-rs-cli
```

<details close>
<summary><b>Additional installation methods</b></summary>

Install dora with our standalone installers, or from [crates.io](https://crates.io/crates/dora-cli):

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

### With Source

```bash
git clone https://github.com/dora-rs/dora.git
cd dora
cargo build --release -p dora-cli
PATH=$PATH:$(pwd)/target/release
```

</details>

### Run

1. Run some Python examples:

```bash
uv venv --seed -p 3.11
dora build https://raw.githubusercontent.com/dora-rs/dora/refs/heads/main/examples/object-detection/yolo.yml --uv
dora run yolo.yml --uv
```

> Make sure to have a webcam

To stop your dataflow, you can use <kbd>ctrl</kbd>+<kbd>c</kbd>

### Documentation

The full documentation is available on [our website](https://dora-rs.ai/).
A lot of guides are available on [this section](https://dora-rs.ai/docs/guides/) of our website.

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

## Support Matrix

|                                   | dora-rs                                                                                 | Hoped for                                                       |
| --------------------------------- | --------------------------------------------------------------------------------------- | --------------------------------------------------------------- |
| **Tier 1 Support**                | Python, Rust                                                                            | C, C++, ROS 2                                                   |
| **Tier 2 Support**                | C, C++, ROS2                                                                            |
| **Message Format**                | Arrow                                                                                   | Native                                                          |
| **Local Communication**           | Shared Memory, [Cuda zero-copy IPC](https://arrow.apache.org/docs/python/api/cuda.html) | Custom Middleware, intra-process `tokio::channel` communication |
| **Remote Communication**          | [Zenoh](https://zenoh.io/)                                                              | Custom Middleware                                               |
| **Metrics, Tracing, and Logging** | Opentelemetry                                                                           | Native logging libraries into Opentelemetry                     |
| **Supported Platforms (x86)**     | Windows, MacOS, Linux                                                                   |
| **Supported Platforms (ARM)**     | MacOS, Linux                                                                            |
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
