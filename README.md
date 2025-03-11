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

## Highlights

- 🚀 dora-rs is a framework to run realtime multi-AI and multi-hardware applications.
- 🦀 dora-rs internals are 100% Rust making it extremely fast compared to alternative such as being ⚡️ [10-17x faster](https://github.com/dora-rs/dora-benchmark) than `ros2`.
- ❇️ Includes a large set of pre-packaged nodes for fast prototyping which simplifies integration of hardware, algorithms, and AI models.

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

- \[03/05\] dora-rs has been accepted to [**GSoC 2025 🎉**](https://summerofcode.withgoogle.com/programs/2025/organizations/dora-rs-tb), with the following [**idea list**](https://github.com/dora-rs/dora/wiki/GSoC_2025).
- \[03/04\] Add support for Zenoh for distributed dataflow.
- \[03/04\] Add support for Meta SAM2, Kokoro(TTS), Improved Qwen2.5 Performance using `llama.cpp`.
- \[02/25\] Add support for Qwen2.5(LLM), Qwen2.5-VL(VLM), outetts(TTS)
</details>

## Support Matrix

|                                   | dora-rs                                                                                                                                                                                  |
| --------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **APIs**                          | Python >= 3.7 ✅ <br> Rust ✅<br> C/C++ 🆗 <br>ROS2 >= Foxy 🆗                                                                                                                           |
| **OS**                            | Linux: Arm 32 ✅ Arm 64 ✅ x64_86 ✅ <br>MacOS: Arm 64 ✅ x64_86 ✅<br>Windows: x64_86 🆗<br> Android: 🛠️ (Blocked by: https://github.com/elast0ny/shared_memory/issues/32) <br> IOS: 🛠️ |
| **Message Format**                | Arrow ✅ <br> Standard Specification 🛠️                                                                                                                                                  |
| **Local Communication**           | Shared Memory ✅ <br> [Cuda IPC](https://arrow.apache.org/docs/python/api/cuda.html) 📐                                                                                                  |
| **Remote Communication**          | [Zenoh](https://zenoh.io/) 📐                                                                                                                                                            |
| **Metrics, Tracing, and Logging** | Opentelemetry 📐                                                                                                                                                                         |
| **Configuration**                 | YAML ✅                                                                                                                                                                                  |
| **Package Manager**               | [pip](https://pypi.org/): Python Node ✅ Rust Node ✅ C/C++ Node 🛠️ <br>[cargo](https://crates.io/): Rust Node ✅                                                                        |

> - ✅ = First Class Support
> - 🆗 = Best Effort Support
> - 📐 = Experimental and looking for contributions
> - 🛠️ = Unsupported but hoped for through contributions
>
> Everything is open for contributions 🙋

## Node Hub

> Feel free to modify this README with your own nodes so that it benefits the community.

### Camera

| Title                                                                                    | Support            | Description                         | Downloads                                                         | License                                                        | Release                                                        |
| ---------------------------------------------------------------------------------------- | ------------------ | ----------------------------------- | ----------------------------------------------------------------- | -------------------------------------------------------------- | -------------------------------------------------------------- |
| [PyOrbbeckSDK](https://github.com/dora-rs/dora/blob/main/node-hub/dora-pyorbbecksdk)     | 📐                 | Image and depth from Orbbeck Camera | ![Downloads](https://img.shields.io/pypi/dm/dora-pyorbbecksdk)    | ![License](https://img.shields.io/pypi/l/dora-pyorbbecksdk)    | ![Release](https://img.shields.io/pypi/v/dora-pyorbbecksdk)    |
| [PyRealsense](https://github.com/dora-rs/dora/blob/main/node-hub/dora-pyrealsense)       | Linux🆗 <br> Mac🛠️ | Image and depth from Realsense      | ![Downloads](https://img.shields.io/pypi/dm/dora-pyrealsense)     | ![License](https://img.shields.io/pypi/l/dora-pyrealsense)     | ![Release](https://img.shields.io/pypi/v/dora-pyrealsense)     |
| [Video Capture](https://github.com/dora-rs/dora/blob/main/node-hub/opencv-video-capture) | ✅                 | Image stream from Camera            | ![Downloads](https://img.shields.io/pypi/dm/opencv-video-capture) | ![License](https://img.shields.io/pypi/l/opencv-video-capture) | ![Release](https://img.shields.io/pypi/v/opencv-video-capture) |

### Peripheral

| Title                                                                               | Support | Description               | Downloads                                                    | License                                                   | Release                                                   |
| ----------------------------------------------------------------------------------- | ------- | ------------------------- | ------------------------------------------------------------ | --------------------------------------------------------- | --------------------------------------------------------- |
| [Keyboard](https://github.com/dora-rs/dora/blob/main/node-hub/dora-keyboard)        | ✅      | Keyboard char listener    | ![Downloads](https://img.shields.io/pypi/dm/dora-keyboard)   | ![License](https://img.shields.io/pypi/l/dora-keyboard)   | ![Release](https://img.shields.io/pypi/v/dora-keyboard)   |
| [Microphone](https://github.com/dora-rs/dora/blob/main/node-hub/dora-microphone)    | ✅      | Audio from microphone     | ![Downloads](https://img.shields.io/pypi/dm/dora-microphone) | ![License](https://img.shields.io/pypi/l/dora-microphone) | ![Release](https://img.shields.io/pypi/v/dora-microphone) |
| [PyAudio(Speaker)](https://github.com/dora-rs/dora/blob/main/node-hub/dora-pyaudio) | ✅      | Output audio from speaker | ![Downloads](https://img.shields.io/pypi/dm/dora-pyaudio)    | ![License](https://img.shields.io/pypi/l/dora-pyaudio)    | ![Release](https://img.shields.io/pypi/v/dora-pyaudio)    |

### Actuator

| Title                                                                                    | Support | Description      | Downloads | License | Release |
| ---------------------------------------------------------------------------------------- | ------- | ---------------- | --------- | ------- | ------- |
| [Feetech](https://github.com/dora-rs/dora-lerobot/blob/main/node-hub/feetech-client)     | 📐      | Feetech Client   |           |         |         |
| [Dynamixel](https://github.com/dora-rs/dora-lerobot/blob/main/node-hub/dynamixel-client) | 📐      | Dynamixel Client |           |         |         |

### Chassis

| Title                                                                           | Support | Description         | Downloads                                                 | License                                                | Release                                                |
| ------------------------------------------------------------------------------- | ------- | ------------------- | --------------------------------------------------------- | ------------------------------------------------------ | ------------------------------------------------------ |
| [Agilex - UGV](https://github.com/dora-rs/dora/blob/main/node-hub/dora-ugv)     | 🆗      | Robomaster Client   | ![Downloads](https://img.shields.io/pypi/dm/dora-ugv)     | ![License](https://img.shields.io/pypi/l/dora-ugv)     | ![Release](https://img.shields.io/pypi/v/dora-ugv)     |
| [DJI - Robomaster S1](https://huggingface.co/datasets/dora-rs/dora-robomaster)  | 📐      | Robomaster Client   |                                                           |                                                        |                                                        |
| [Dora Kit Car](https://github.com/dora-rs/dora/blob/main/node-hub/dora-kit-car) | 🆗      | Open Source Chassis | ![Downloads](https://img.shields.io/pypi/dm/dora-kit-car) | ![License](https://img.shields.io/pypi/l/dora-kit-car) | ![Release](https://img.shields.io/pypi/v/dora-kit-car) |

### Arm

| Title                                                                                            | Support | Description                       | Downloads                                               | License                                              | Release                                              |
| ------------------------------------------------------------------------------------------------ | ------- | --------------------------------- | ------------------------------------------------------- | ---------------------------------------------------- | ---------------------------------------------------- |
| [Alex Koch - Low Cost Robot](https://github.com/dora-rs/dora-lerobot/blob/main/robots/alexk-lcr) | 📐      | Alex Koch - Low Cost Robot Client |                                                         |                                                      |                                                      |
| [Lebai - LM3](https://github.com/dora-rs/dora-lerobot/blob/main/node-hub/lebai-client)           | 📐      | Lebai client                      |                                                         |                                                      |                                                      |
| [Agilex - Piper](https://github.com/dora-rs/dora/blob/main/node-hub/dora-piper)                  | 🆗      | Agilex arm client                 | ![Downloads](https://img.shields.io/pypi/dm/dora-piper) | ![License](https://img.shields.io/pypi/l/dora-piper) | ![Release](https://img.shields.io/pypi/v/dora-piper) |

### Robot

| Title                                                                                        | Support | Description     | Downloads                                                 | License                                                | Release                                                |
| -------------------------------------------------------------------------------------------- | ------- | --------------- | --------------------------------------------------------- | ------------------------------------------------------ | ------------------------------------------------------ |
| [Pollen - Reachy 1](https://github.com/dora-rs/dora-lerobot/blob/main/node-hub/dora-reachy1) | 📐      | Reachy 1 Client |                                                           |                                                        |                                                        |
| [Pollen - Reachy 2](https://github.com/dora-rs/dora/blob/main/node-hub/dora-reachy2)         | 🆗      | Reachy 2 client | ![Downloads](https://img.shields.io/pypi/dm/dora-reachy2) | ![License](https://img.shields.io/pypi/l/dora-reachy2) | ![Release](https://img.shields.io/pypi/v/dora-reachy2) |
| [Trossen - Aloha](https://github.com/dora-rs/dora-lerobot/blob/main/robots/aloha)            | 📐      | Aloha client    |                                                           |                                                        |                                                        |

### Voice Activity Detection(VAD)

| Title                                                                     | Support | Description                     | Downloads                                             | License                                            | Release                                            |
| ------------------------------------------------------------------------- | ------- | ------------------------------- | ----------------------------------------------------- | -------------------------------------------------- | -------------------------------------------------- |
| [Silero VAD](https://github.com/dora-rs/dora/blob/main/node-hub/dora-vad) | ✅      | Silero Voice activity detection | ![Downloads](https://img.shields.io/pypi/dm/dora-vad) | ![License](https://img.shields.io/pypi/l/dora-vad) | ![Release](https://img.shields.io/pypi/v/dora-vad) |

### Speech to Text(STT)

| Title                                                                             | Support | Description              | Downloads                                                        | License                                                       | Release                                                       |
| --------------------------------------------------------------------------------- | ------- | ------------------------ | ---------------------------------------------------------------- | ------------------------------------------------------------- | ------------------------------------------------------------- |
| [Whisper](https://github.com/dora-rs/dora/blob/main/node-hub/dora-distil-whisper) | ✅      | Transcribe audio to text | ![Downloads](https://img.shields.io/pypi/dm/dora-distil-whisper) | ![License](https://img.shields.io/pypi/l/dora-distil-whisper) | ![Release](https://img.shields.io/pypi/v/dora-distil-whisper) |

### Object Detection

| Title                                                                  | Support | Description      | Downloads                                              | License                                             | Release                                             |
| ---------------------------------------------------------------------- | ------- | ---------------- | ------------------------------------------------------ | --------------------------------------------------- | --------------------------------------------------- |
| [Yolov8](https://github.com/dora-rs/dora/blob/main/node-hub/dora-yolo) | ✅      | Object detection | ![Downloads](https://img.shields.io/pypi/dm/dora-yolo) | ![License](https://img.shields.io/pypi/l/dora-yolo) | ![Release](https://img.shields.io/pypi/v/dora-yolo) |

### Segmentation

| Title                                                                | Support             | Description      | Downloads                                              | License                                             | Release                                             |
| -------------------------------------------------------------------- | ------------------- | ---------------- | ------------------------------------------------------ | --------------------------------------------------- | --------------------------------------------------- |
| [SAM2](https://github.com/dora-rs/dora/blob/main/node-hub/dora-sam2) | Cuda✅ <br> Metal🛠️ | Segment Anything | ![Downloads](https://img.shields.io/pypi/dm/dora-sam2) | ![License](https://img.shields.io/pypi/l/dora-sam2) | ![Release](https://img.shields.io/pypi/v/dora-sam2) |

### Large Language Model(LLM)

| Title                                                                   | Support | Description                     | Downloads                                              | License                                             | Release                                             |
| ----------------------------------------------------------------------- | ------- | ------------------------------- | ------------------------------------------------------ | --------------------------------------------------- | --------------------------------------------------- |
| [Qwen2.5](https://github.com/dora-rs/dora/blob/main/node-hub/dora-qwen) | ✅      | Large Language Model using Qwen | ![Downloads](https://img.shields.io/pypi/dm/dora-qwen) | ![License](https://img.shields.io/pypi/l/dora-qwen) | ![Release](https://img.shields.io/pypi/v/dora-qwen) |

### Vision Language Model(VLM)

| Title                                                                            | Support | Description                            | Downloads                                                    | License                                                   | Release                                                   |
| -------------------------------------------------------------------------------- | ------- | -------------------------------------- | ------------------------------------------------------------ | --------------------------------------------------------- | --------------------------------------------------------- |
| [Qwen2.5-vl](https://github.com/dora-rs/dora/blob/main/node-hub/dora-qwen2-5-vl) | ✅      | Vision Language Model using Qwen2.5 VL | ![Downloads](https://img.shields.io/pypi/dm/dora-qwen2-5-vl) | ![License](https://img.shields.io/pypi/l/dora-qwen2-5-vl) | ![Release](https://img.shields.io/pypi/v/dora-qwen2-5-vl) |
| [InternVL](https://github.com/dora-rs/dora/blob/main/node-hub/dora-internvl)     | 🆗      | InternVL is a vision language model    | ![Downloads](https://img.shields.io/pypi/dm/dora-internvl)   | ![License](https://img.shields.io/pypi/l/dora-internvl)   | ![Release](https://img.shields.io/pypi/v/dora-internvl)   |

### Vision Language Action(VLA)

| Title                                                                    | Support | Description                                      | Downloads                                                | License                                               | Release                                               |
| ------------------------------------------------------------------------ | ------- | ------------------------------------------------ | -------------------------------------------------------- | ----------------------------------------------------- | ----------------------------------------------------- |
| [RDT-1B](https://github.com/dora-rs/dora/blob/main/node-hub/dora-rdt-1b) | 🆗      | Infer policy using Robotic Diffusion Transformer | ![Downloads](https://img.shields.io/pypi/dm/dora-rdt-1b) | ![License](https://img.shields.io/pypi/l/dora-rdt-1b) | ![Release](https://img.shields.io/pypi/v/dora-rdt-1b) |

### Translation

| Title                                                                                   | Support | Description                     | Downloads                                                       | License                                                      | Release                                                      |
| --------------------------------------------------------------------------------------- | ------- | ------------------------------- | --------------------------------------------------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| [ArgosTranslate](https://github.com/dora-rs/dora/blob/main/node-hub/dora-argotranslate) | 🆗      | Open Source translation engine  | ![Downloads](https://img.shields.io/pypi/dm/dora-argotranslate) | ![License](https://img.shields.io/pypi/l/dora-argotranslate) | ![Release](https://img.shields.io/pypi/v/dora-argotranslate) |
| [Opus MT](https://github.com/dora-rs/dora/blob/main/node-hub/dora-opus)                 | 🆗      | Translate text between language | ![Downloads](https://img.shields.io/pypi/dm/dora-opus)          | ![License](https://img.shields.io/pypi/l/dora-opus)          | ![Release](https://img.shields.io/pypi/v/dora-opus)          |

### Text to Speech(TTS)

| Title                                                                            | Support | Description              | Downloads                                                    | License                                                   | Release                                                   |
| -------------------------------------------------------------------------------- | ------- | ------------------------ | ------------------------------------------------------------ | --------------------------------------------------------- | --------------------------------------------------------- |
| [Kokoro TTS](https://github.com/dora-rs/dora/blob/main/node-hub/dora-kokoro-tts) | ✅      | Efficient Text to Speech | ![Downloads](https://img.shields.io/pypi/dm/dora-kokoro-tts) | ![License](https://img.shields.io/pypi/l/dora-kokoro-tts) | ![Release](https://img.shields.io/pypi/v/dora-kokoro-tts) |

### Recorder

| Title                                                                                               | Support | Description                      | Downloads                                                           | License                                                          | Release                                                          |
| --------------------------------------------------------------------------------------------------- | ------- | -------------------------------- | ------------------------------------------------------------------- | ---------------------------------------------------------------- | ---------------------------------------------------------------- |
| [Llama Factory Recorder](https://github.com/dora-rs/dora/blob/main/node-hub/llama-factory-recorder) | 🆗      | Record data to train LLM and VLM | ![Downloads](https://img.shields.io/pypi/dm/llama-factory-recorder) | ![License](https://img.shields.io/pypi/l/llama-factory-recorder) | ![Release](https://img.shields.io/pypi/v/llama-factory-recorder) |
| [LeRobot Recorder](https://github.com/dora-rs/dora-lerobot/blob/main/node-hub/lerobot-dashboard)    | 📐      | LeRobot Recorder helper          |                                                                     |                                                                  |                                                                  |

### Visualization

| Title                                                                  | Support | Description                      | Downloads                                               | License                                               | Release                                               |
| ---------------------------------------------------------------------- | ------- | -------------------------------- | ------------------------------------------------------- | ----------------------------------------------------- | ----------------------------------------------------- |
| [Plot](https://github.com/dora-rs/dora/blob/main/node-hub/opencv-plot) | ✅      | Simple OpenCV plot visualization | ![Downloads](https://img.shields.io/pypi/dm/dora-yolo)  | ![License](https://img.shields.io/pypi/l/opencv-plot) | ![Release](https://img.shields.io/pypi/v/opencv-plot) |
| [Rerun](https://github.com/dora-rs/dora/blob/main/node-hub/dora-rerun) | ✅      | Visualization tool               | ![Downloads](https://img.shields.io/pypi/dm/dora-rerun) | ![License](https://img.shields.io/pypi/l/dora-rerun)  | ![Release](https://img.shields.io/pypi/v/dora-rerun)  |

### Simulator

| Title                                                                              | Support | Description                          | Downloads | License | Release |
| ---------------------------------------------------------------------------------- | ------- | ------------------------------------ | --------- | ------- | ------- |
| [Mujoco](https://github.com/dora-rs/dora-lerobot/blob/main/node-hub/mujoco-client) | 📐      | Mujoco Simulator                     |           |         |         |
| [Carla](https://github.com/dora-rs/dora-drives)                                    | 📐      | Carla Simulator                      |           |         |         |
| [Gymnasium](https://github.com/dora-rs/dora-lerobot/blob/main/gym_dora)            | 📐      | Experimental OpenAI Gymnasium bridge |           |         |         |

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

### With Docker

You can also run `dora-rs` using Docker. Here are the steps to get started:

1. Build the Docker image using Dockerfile:

```bash
# Inside your dora repo
docker build -t dora-rs/dora-rs-cli .
```

2. Run the Docker container:

```bash
docker run -it --rm dora-rs/dora-cli
```

3. Inside the Docker container, you can run the yolo python example:

</details>

### Run

- Run the yolo python example:

```bash
## Create a virtual environment
uv venv --seed -p 3.11
```
> Note: If you are using Docker, you do not need to create a virtual environment.

## Install nodes dependencies of a remote graph
dora build https://raw.githubusercontent.com/dora-rs/dora/refs/heads/main/examples/object-detection/yolo.yml --uv

## Run yolo graph
dora run yolo.yml --uv
```

> Make sure to have a webcam

To stop your dataflow, you can use <kbd>ctrl</kbd>+<kbd>c</kbd>

- To understand what is happening, you can look at the dataflow with:

```bash
cat yolo.yml
```

- Resulting in:

```yaml
nodes:
  - id: camera
    build: pip install opencv-video-capture
    path: opencv-video-capture
    inputs:
      tick: dora/timer/millis/20
    outputs:
      - image
    env:
      CAPTURE_PATH: 0
      IMAGE_WIDTH: 640
      IMAGE_HEIGHT: 480

  - id: object-detection
    build: pip install dora-yolo
    path: dora-yolo
    inputs:
      image: camera/image
    outputs:
      - bbox

  - id: plot
    build: pip install dora-rerun
    path: dora-rerun
    inputs:
      image: camera/image
      boxes2d: object-detection/bbox
```

- In the above example, we can understand that the camera is sending image to both the rerun viewer as well as a yolo model that generates bounding box that is visualized within rerun.

### Documentation

The full documentation is available on [our website](https://dora-rs.ai/).
A lot of guides are available on [this section](https://dora-rs.ai/docs/guides/) of our website.

## What is Dora? And what features does Dora offer?

**D**ataflow-**O**riented **R**obotic **A**rchitecture (`dora-rs`) is a framework that makes creation of robotic applications fast and simple.

`dora-rs` implements a declarative dataflow paradigm where tasks are split between nodes isolated as individual processes.

The dataflow paradigm has the advantage of creating an abstraction layer that makes robotic applications modular and easily configurable.

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
