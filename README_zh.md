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

简体中文 [English](./README.md)

## 亮点

*   🚀 `dora-rs` 是一个运行实时多`AI`和多硬件应用的框架。
*   🦀 `dora-rs` 的核心 `100%` 使用 `Rust` 语言开发，相比同类框架具有极速性能，比`ros2`快 ⚡️ [10-17倍](https://github.com/dora-rs/dora-benchmark)。
*   ❇️ 内置大量预封装节点，支持快速原型开发，可简化硬件、算法与`AI`模型的集成流程。


<p align="center">
  <picture align="center">
    <source media="(prefers-color-scheme: dark)" srcset="https://raw.githubusercontent.com/dora-rs/dora/main/docs/src/bar_chart_dark.svg">
    <source media="(prefers-color-scheme: light)" srcset="https://raw.githubusercontent.com/dora-rs/dora/main/docs/src/bar_chart_light.svg">
    <img src="https://raw.githubusercontent.com/dora-rs/dora/main/docs/src/bar_chart_light.svg">
  </picture>
</p>

<p align="center">
 <a href="https://github.com/dora-rs/dora-benchmark/" >
  <i>针对两个框架的 Python API 进行延迟基准测试（发送40兆随机字节数据）。</i>
  </a>
</p>

## 最新动态 🎉

<details open>
<summary><b>2025</b></summary>

\[04/05\] 新增功能支持：dora-cotracker：实现视频帧任意点位追踪，dora-rav1e：支持12bit高色深AV1编码，dora-dav1d：完成AV1格式解码支持

- \[03/05\] 新增 dora 异步 Python 支持
- \[03/05\] 新增 Microsoft Phi4 和 Microsoft Magma 支持
- \[03/05\] dora-rs 成功入选 [**GSoC 2025 🎉**](https://summerofcode.withgoogle.com/programs/2025/organizations/dora-rs-tb)，并公布以下项目 [**创意列表**](https://github.com/dora-rs/dora/wiki/GSoC_2025) 
- \[03/04\] 新增 Zenoh 分布式数据流支持
- \[03/04\] 新增 Meta SAM2、Kokoro（TTS）支持，并基于 `llama.cpp` 优化 Qwen2.5 性能表现
- \[02/25\] 新增 Qwen2.5（大语言模型）、Qwen2.5-VL（视觉语言模型）及 outetts（TTS语音合成）支持
</details>

## 支持矩阵

|                                   | dora-rs                                                                                                                                                                                          |
| --------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| **APIs**                          | Python >= 3.7 including sync ⭐✅ <br> Rust ✅<br> C/C++ 🆗 <br>ROS2 >= Foxy 🆗                                                                                                                  |
| **OS**                            | Linux: Arm 32 ⭐✅ Arm 64 ⭐✅ x64_86 ⭐✅ <br>MacOS: Arm 64 ⭐✅ x64_86 ✅<br>Windows: x64_86 🆗<br> Android: 🛠️ (Blocked by: https://github.com/elast0ny/shared_memory/issues/32) <br> IOS: 🛠️ |
| **Message Format**                | Arrow ✅ <br> Standard Specification 🛠️                                                                                                                                                          |
| **Local Communication**           | Shared Memory ✅ <br> [Cuda IPC](https://arrow.apache.org/docs/python/api/cuda.html) 📐                                                                                                          |
| **Remote Communication**          | [Zenoh](https://zenoh.io/) 📐                                                                                                                                                                    |
| **Metrics, Tracing, and Logging** | Opentelemetry 📐                                                                                                                                                                                 |
| **Configuration**                 | YAML ✅                                                                                                                                                                                          |
| **Package Manager**               | [pip](https://pypi.org/): Python Node ✅ Rust Node ✅ C/C++ Node 🛠️ <br>[cargo](https://crates.io/): Rust Node ✅                                                                                |

> - ⭐ = 推荐方案
> - ✅ = 一级支持（官方全面支持）
> - 🆗 = 尽力支持（社区主导支持）
> - 📐 = 实验性功能（期待贡献）
> - 🛠️ = 暂不支持（期待社区贡献实现）
>
> 开放共建，欢迎来PR！ 🙋

## Node Hub

> Feel free to modify this README with your own nodes so that it benefits the community.

| Type                          | Title                                                                                               | Support             | Description                                      | Downloads                                                                     | License                                                                    |
| ----------------------------- | --------------------------------------------------------------------------------------------------- | ------------------- | ------------------------------------------------ | ----------------------------------------------------------------------------- | -------------------------------------------------------------------------- |
| Camera                        | [PyOrbbeckSDK](https://github.com/dora-rs/dora/blob/main/node-hub/dora-pyorbbecksdk)                | 📐                  | Image and depth from Orbbeck Camera              | ![Downloads](https://img.shields.io/pypi/dm/dora-pyorbbecksdk?label=%20)      | ![License](https://img.shields.io/pypi/l/dora-pyorbbecksdk?label=%20)      |
| Camera                        | [PyRealsense](https://github.com/dora-rs/dora/blob/main/node-hub/dora-pyrealsense)                  | Linux🆗 <br> Mac🛠️  | Image and depth from Realsense                   | ![Downloads](https://img.shields.io/pypi/dm/dora-pyrealsense?label=%20)       | ![License](https://img.shields.io/pypi/l/dora-pyrealsense?label=%20)       |
| Camera                        | [OpenCV Video Capture](https://github.com/dora-rs/dora/blob/main/node-hub/opencv-video-capture)     | ✅                  | Image stream from OpenCV Camera                  | ![Downloads](https://img.shields.io/pypi/dm/opencv-video-capture?label=%20)   | ![License](https://img.shields.io/pypi/l/opencv-video-capture?label=%20)   |
| Peripheral                    | [Keyboard](https://github.com/dora-rs/dora/blob/main/node-hub/dora-keyboard)                        | ✅                  | Keyboard char listener                           | ![Downloads](https://img.shields.io/pypi/dm/dora-keyboard?label=%20)          | ![License](https://img.shields.io/pypi/l/dora-keyboard?label=%20)          |
| Peripheral                    | [Microphone](https://github.com/dora-rs/dora/blob/main/node-hub/dora-microphone)                    | ✅                  | Audio from microphone                            | ![Downloads](https://img.shields.io/pypi/dm/dora-microphone?label=%20)        | ![License](https://img.shields.io/pypi/l/dora-microphone?label=%20)        |
| Peripheral                    | [PyAudio(Speaker)](https://github.com/dora-rs/dora/blob/main/node-hub/dora-pyaudio)                 | ✅                  | Output audio from speaker                        | ![Downloads](https://img.shields.io/pypi/dm/dora-pyaudio?label=%20)           | ![License](https://img.shields.io/pypi/l/dora-pyaudio?label=%20)           |
| Actuator                      | [Feetech](https://github.com/dora-rs/dora-lerobot/blob/main/node-hub/feetech-client)                | 📐                  | Feetech Client                                   |                                                                               |                                                                            |
| Actuator                      | [Dynamixel](https://github.com/dora-rs/dora-lerobot/blob/main/node-hub/dynamixel-client)            | 📐                  | Dynamixel Client                                 |                                                                               |                                                                            |
| Chassis                       | [Agilex - UGV](https://github.com/dora-rs/dora/blob/main/node-hub/dora-ugv)                         | 🆗                  | Robomaster Client                                | ![Downloads](https://img.shields.io/pypi/dm/dora-ugv?label=%20)               | ![License](https://img.shields.io/pypi/l/dora-ugv?label=%20)               |
| Chassis                       | [DJI - Robomaster S1](https://huggingface.co/datasets/dora-rs/dora-robomaster)                      | 📐                  | Robomaster Client                                |                                                                               |                                                                            |
| Chassis                       | [Dora Kit Car](https://github.com/dora-rs/dora/blob/main/node-hub/dora-kit-car)                     | 🆗                  | Open Source Chassis                              | ![Downloads](https://img.shields.io/pypi/dm/dora-kit-car?label=%20)           | ![License](https://img.shields.io/pypi/l/dora-kit-car?label=%20)           |
| Arm                           | [Alex Koch - Low Cost Robot](https://github.com/dora-rs/dora-lerobot/blob/main/robots/alexk-lcr)    | 📐                  | Alex Koch - Low Cost Robot Client                |                                                                               |                                                                            |
| Arm                           | [Lebai - LM3](https://github.com/dora-rs/dora-lerobot/blob/main/node-hub/lebai-client)              | 📐                  | Lebai client                                     |                                                                               |                                                                            |
| Arm                           | [Agilex - Piper](https://github.com/dora-rs/dora/blob/main/node-hub/dora-piper)                     | 🆗                  | Agilex arm client                                | ![Downloads](https://img.shields.io/pypi/dm/dora-piper?label=%20)             | ![License](https://img.shields.io/pypi/l/dora-piper?label=%20)             |
| Robot                         | [Pollen - Reachy 1](https://github.com/dora-rs/dora-lerobot/blob/main/node-hub/dora-reachy1)        | 📐                  | Reachy 1 Client                                  |                                                                               |                                                                            |
| Robot                         | [Pollen - Reachy 2](https://github.com/dora-rs/dora/blob/main/node-hub/dora-reachy2)                | 🆗                  | Reachy 2 client                                  | ![Downloads](https://img.shields.io/pypi/dm/dora-reachy2?label=%20)           | ![License](https://img.shields.io/pypi/l/dora-reachy2?label=%20)           |
| Robot                         | [Trossen - Aloha](https://github.com/dora-rs/dora-lerobot/blob/main/robots/aloha)                   | 📐                  | Aloha client                                     |                                                                               |                                                                            |
| Voice Activity Detection(VAD) | [Silero VAD](https://github.com/dora-rs/dora/blob/main/node-hub/dora-vad)                           | ✅                  | Silero Voice activity detection                  | ![Downloads](https://img.shields.io/pypi/dm/dora-vad?label=%20)               | ![License](https://img.shields.io/pypi/l/dora-vad?label=%20)               |
| Speech to Text(STT)           | [Whisper](https://github.com/dora-rs/dora/blob/main/node-hub/dora-distil-whisper)                   | ✅                  | Transcribe audio to text                         | ![Downloads](https://img.shields.io/pypi/dm/dora-distil-whisper?label=%20)    | ![License](https://img.shields.io/pypi/l/dora-distil-whisper?label=%20)    |
| Object Detection              | [Yolov8](https://github.com/dora-rs/dora/blob/main/node-hub/dora-yolo)                              | ✅                  | Object detection                                 | ![Downloads](https://img.shields.io/pypi/dm/dora-yolo?label=%20)              | ![License](https://img.shields.io/pypi/l/dora-yolo?label=%20)              |
| Segmentation                  | [SAM2](https://github.com/dora-rs/dora/blob/main/node-hub/dora-sam2)                                | Cuda✅ <br> Metal🛠️ | Segment Anything                                 | ![Downloads](https://img.shields.io/pypi/dm/dora-sam2?label=%20)              | ![License](https://img.shields.io/pypi/l/dora-sam2?label=%20)              |
| Large Language Model(LLM)     | [Qwen2.5](https://github.com/dora-rs/dora/blob/main/node-hub/dora-qwen)                             | ✅                  | Large Language Model using Qwen                  | ![Downloads](https://img.shields.io/pypi/dm/dora-qwen?label=%20)              | ![License](https://img.shields.io/pypi/l/dora-qwen?label=%20)              |
| Vision Language Model(VLM)    | [Qwen2.5-vl](https://github.com/dora-rs/dora/blob/main/node-hub/dora-qwen2-5-vl)                    | ✅                  | Vision Language Model using Qwen2.5 VL           | ![Downloads](https://img.shields.io/pypi/dm/dora-qwen2-5-vl?label=%20)        | ![License](https://img.shields.io/pypi/l/dora-qwen2-5-vl?label=%20)        |
| Vision Language Model(VLM)    | [InternVL](https://github.com/dora-rs/dora/blob/main/node-hub/dora-internvl)                        | 🆗                  | InternVL is a vision language model              | ![Downloads](https://img.shields.io/pypi/dm/dora-internvl?label=%20)          | ![License](https://img.shields.io/pypi/l/dora-internvl?label=%20)          |
| Vision Language Action(VLA)   | [RDT-1B](https://github.com/dora-rs/dora/blob/main/node-hub/dora-rdt-1b)                            | 🆗                  | Infer policy using Robotic Diffusion Transformer | ![Downloads](https://img.shields.io/pypi/dm/dora-rdt-1b?label=%20)            | ![License](https://img.shields.io/pypi/l/dora-rdt-1b?label=%20)            |
| Translation                   | [ArgosTranslate](https://github.com/dora-rs/dora/blob/main/node-hub/dora-argotranslate)             | 🆗                  | Open Source translation engine                   | ![Downloads](https://img.shields.io/pypi/dm/dora-argotranslate?label=%20)     | ![License](https://img.shields.io/pypi/l/dora-argotranslate?label=%20)     |
| Translation                   | [Opus MT](https://github.com/dora-rs/dora/blob/main/node-hub/dora-opus)                             | 🆗                  | Translate text between language                  | ![Downloads](https://img.shields.io/pypi/dm/dora-opus?label=%20)              | ![License](https://img.shields.io/pypi/l/dora-opus?label=%20)              |
| Text to Speech(TTS)           | [Kokoro TTS](https://github.com/dora-rs/dora/blob/main/node-hub/dora-kokoro-tts)                    | ✅                  | Efficient Text to Speech                         | ![Downloads](https://img.shields.io/pypi/dm/dora-kokoro-tts?label=%20)        | ![License](https://img.shields.io/pypi/l/dora-kokoro-tts?label=%20)        |
| Recorder                      | [Llama Factory Recorder](https://github.com/dora-rs/dora/blob/main/node-hub/llama-factory-recorder) | 🆗                  | Record data to train LLM and VLM                 | ![Downloads](https://img.shields.io/pypi/dm/llama-factory-recorder?label=%20) | ![License](https://img.shields.io/pypi/l/llama-factory-recorder?label=%20) |
| Recorder                      | [LeRobot Recorder](https://github.com/dora-rs/dora-lerobot/blob/main/node-hub/lerobot-dashboard)    | 📐                  | LeRobot Recorder helper                          |                                                                               |                                                                            |
| Visualization                 | [Plot](https://github.com/dora-rs/dora/blob/main/node-hub/opencv-plot)                              | ✅                  | Simple OpenCV plot visualization                 | ![Downloads](https://img.shields.io/pypi/dm/dora-yolo?label=%20)              | ![License](https://img.shields.io/pypi/l/opencv-plot?label=%20)            |
| Visualization                 | [Rerun](https://github.com/dora-rs/dora/blob/main/node-hub/dora-rerun)                              | ✅                  | Visualization tool                               | ![Downloads](https://img.shields.io/pypi/dm/dora-rerun?label=%20)             | ![License](https://img.shields.io/pypi/l/dora-rerun?label=%20)             |
| Simulator                     | [Mujoco](https://github.com/dora-rs/dora-lerobot/blob/main/node-hub/mujoco-client)                  | 📐                  | Mujoco Simulator                                 |                                                                               |                                                                            |
| Simulator                     | [Carla](https://github.com/dora-rs/dora-drives)                                                     | 📐                  | Carla Simulator                                  |                                                                               |                                                                            |
| Simulator                     | [Gymnasium](https://github.com/dora-rs/dora-lerobot/blob/main/gym_dora)                             | 📐                  | Experimental OpenAI Gymnasium bridge             |                                                                               |                                                                            |

## Examples

| Type           | Title                                                                                                        | Description                                         | Last Commit                                                                                                        |
| -------------- | ------------------------------------------------------------------------------------------------------------ | --------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------ |
| Audio          | [Speech to Text(STT)](https://github.com/dora-rs/dora/blob/main/examples/speech-to-text)                     | Transform speech to text.                           | ![License](https://img.shields.io/github/last-commit/dora-rs/dora?path=examples%2Fspeech-to-text&label=%20)        |
| Audio          | [Translation](https://github.com/dora-rs/dora/blob/main/examples/translation)                                | Translate audio in real time.                       | ![License](https://img.shields.io/github/last-commit/dora-rs/dora?path=examples%2Ftranslation&label=%20)           |
| Vision         | [Vision Language Model(VLM)](https://github.com/dora-rs/dora/blob/main/examples/vlm)                         | Use a VLM to understand images.                     | ![License](https://img.shields.io/github/last-commit/dora-rs/dora?path=examples%2Fvlm&label=%20)                   |
| Vision         | [YOLO](https://github.com/dora-rs/dora/blob/main/examples/python-dataflow)                                   | Use YOLO to detect object within image.             | ![License](https://img.shields.io/github/last-commit/dora-rs/dora?path=examples%2Fpython-dataflow&label=%20)       |
| Vision         | [Camera](https://github.com/dora-rs/dora/blob/main/examples/camera)                                          | Simple webcam plot example                          | ![License](https://img.shields.io/github/last-commit/dora-rs/dora?path=examples%2Fcamera&label=%20)                |
| Model Training | [Piper RDT](https://github.com/dora-rs/dora/blob/main/examples/piper)                                        | Piper RDT Pipeline                                  | ![License](https://img.shields.io/github/last-commit/dora-rs/dora?path=examples%2Fpiper&label=%20)                 |
| Model Training | [LeRobot - Alexander Koch](https://raw.githubusercontent.com/dora-rs/dora-lerobot/refs/heads/main/README.md) | Training Alexander Koch Low Cost Robot with LeRobot | ![License](https://img.shields.io/github/last-commit/dora-rs/dora-lerobot?path=robots&label=%20)                   |
| ROS2           | [C++ ROS2 Example](https://github.com/dora-rs/dora/blob/main/examples/c++-ros2-dataflow)                     | Example using C++ ROS2                              | ![License](https://img.shields.io/github/last-commit/dora-rs/dora?path=examples%2Fc%2b%2b-ros2-dataflow&label=%20) |
| ROS2           | [Rust ROS2 Example](https://github.com/dora-rs/dora/blob/main/examples/rust-ros2-dataflow)                   | Example using Rust ROS2                             | ![License](https://img.shields.io/github/last-commit/dora-rs/dora?path=examples%2Frust-ros2-dataflow&label=%20)    |
| ROS2           | [Python ROS2 Example](https://github.com/dora-rs/dora/blob/main/examples/python-ros2-dataflow)               | Example using Python ROS2                           | ![License](https://img.shields.io/github/last-commit/dora-rs/dora?path=examples%2Fpython-ros2-dataflow&label=%20)  |
| Benchmark      | [GPU Benchmark](https://github.com/dora-rs/dora/blob/main/examples/cuda-benchmark)                           | GPU Benchmark of dora-rs                            | ![License](https://img.shields.io/github/last-commit/dora-rs/dora?path=examples%2Fcuda-benchmark&label=%20)        |
| Benchmark      | [CPU Benchmark](https://github.com/dora-rs/dora-benchmark/blob/main)                                         | CPU Benchmark of dora-rs                            | ![License](https://img.shields.io/github/last-commit/dora-rs/dora-benchmark?path=dora-rs&label=%20)                |
| Tutorial       | [Rust Example](https://github.com/dora-rs/dora/blob/main/examples/rust-dataflow)                             | Example using Rust                                  | ![License](https://img.shields.io/github/last-commit/dora-rs/dora?path=examples%2Frust-dataflow&label=%20)         |
| Tutorial       | [Python Example](https://github.com/dora-rs/dora/blob/main/examples/python-dataflow)                         | Example using Python                                | ![License](https://img.shields.io/github/last-commit/dora-rs/dora?path=examples%2Fpython-dataflow&label=%20)       |
| Tutorial       | [CMake Example](https://github.com/dora-rs/dora/blob/main/examples/cmake-dataflow)                           | Example using CMake                                 | ![License](https://img.shields.io/github/last-commit/dora-rs/dora?path=examples%2Fcmake-dataflow&label=%20)        |
| Tutorial       | [C Example](https://github.com/dora-rs/dora/blob/main/examples/c-dataflow)                                   | Example with C node                                 | ![License](https://img.shields.io/github/last-commit/dora-rs/dora?path=examples%2Fc-dataflow&label=%20)            |
| Tutorial       | [CUDA Example](https://github.com/dora-rs/dora/blob/main/examples/cuda-benchmark)                            | Example using CUDA Zero Copy                        | ![License](https://img.shields.io/github/last-commit/dora-rs/dora?path=examples%2Fcuda-benchmark&label=%20)        |
| Tutorial       | [C++ Example](https://github.com/dora-rs/dora/blob/main/examples/c++-dataflow)                               | Example with C++ node                               | ![License](https://img.shields.io/github/last-commit/dora-rs/dora?path=examples%2Fc%2b%2b-dataflow&label=%20)      |

## 快速开始

### 安装

```bash
pip install dora-rs-cli
```

<details close>
<summary><b>其他安装方式</b></summary>

您可以通过以下两种方式安装 dora, 或者通过 [crates.io](https://crates.io/crates/dora-cli):

### 使用 cargo

```bash
cargo install dora-cli
```

### macOS/Linux 系统用户可通过 GitHub Releases 获取安装包

```bash
curl --proto '=https' --tlsv1.2 -LsSf https://github.com/dora-rs/dora/releases/latest/download/dora-cli-installer.sh | sh
```

### Windows 用户可通过 GitHub Releases 获取安装程序

```powershell
powershell -ExecutionPolicy ByPass -c "irm https://github.com/dora-rs/dorareleases/latest/download/dora-cli-installer.ps1 | iex"
```

### 通过源码

```bash
git clone https://github.com/dora-rs/dora.git
cd dora
cargo build --release -p dora-cli
PATH=$PATH:$(pwd)/target/release
```

</details>

### 运行

- 运行 yolo python 示例:

```bash
## 创建一个虚拟环境
uv venv --seed -p 3.11

## 安装远程计算图的节点依赖
dora build https://raw.githubusercontent.com/dora-rs/dora/refs/heads/main/examples/object-detection/yolo.yml --uv

## 运行 yolo 计算图
dora run yolo.yml --uv
```

> 请确保已连接可用摄像头

停止数据流可以同时按住 <kbd>ctrl</kbd>+<kbd>c</kbd>

- 查看实时数据流运行状态：

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

## Zenoh Integration for Distributed Dataflow (Experimental)

Zenoh is a high-performance pub/sub and query protocol that unifies data in motion and at rest. In **dora-rs**, Zenoh is used for remote communication between nodes running on different machines, enabling distributed dataflow across networks.

### What is Zenoh?

- **Definition:**  
  [Zenoh](https://zenoh.io) is an open-source communication middleware offering pub/sub and query capabilities.
- **Benefits in DORA:**
  - Simplifies communication between distributed nodes.
  - Handles NAT traversal and inter-network communication.
  - Integrates with DORA to manage remote data exchange while local communication still uses efficient shared memory.

### Enabling Zenoh Support

1. **Run a Zenoh Router (`zenohd`):**  
   Launch a Zenoh daemon to mediate communication. For example, using Docker:

   ```bash
   docker run -p 7447:7447 -p 8000:8000 --name zenoh-router eclipse/zenohd:latest
   ```

````markdown
## Create a Zenoh Configuration File 🎛️

Create a file (e.g., `zenoh.json5`) with the router endpoint details:

```json5
{
  connect: {
    endpoints: ["tcp/203.0.113.10:7447"],
  },
}
```
````

---

## Launch DORA Daemons with Zenoh Enabled 🚀

On each machine, export the configuration and start the daemon:

```bash
export ZENOH_CONFIG=/path/to/zenoh.json5
dora daemon --coordinator-addr <COORD_IP> --machine-id <MACHINE_NAME>
```

---

## Deploy Distributed Nodes via YAML 📄

Mark nodes for remote deployment using the `_unstable_deploy` key:

```yaml
nodes:
  - id: camera_node
    outputs: [image]

  - id: processing_node
    _unstable_deploy:
      machine: robot1
      path: /home/robot/dora-nodes/processing_node
    inputs:
      image: camera_node/image
    outputs: [result]
```

---

## Start the Coordinator and Dataflow 🏁

Run the coordinator on a designated machine and start the dataflow:

```bash
dora coordinator
dora start dataflow.yml
```

---

## YAML Example for Distributed Dataflow 📘

```yaml
communication:
  zenoh: {}

nodes:
  - id: camera_node
    custom:
      run: ./camera_driver.py
    outputs:
      - image

  - id: processing_node
    _unstable_deploy:
      machine: robot1
      path: /home/robot/dora-nodes/processing_node
    inputs:
      image: camera_node/image
    outputs:
      - result
```

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

---

## Further Resources 📚

- [Zenoh Documentation](https://zenoh.io/docs/)
- [DORA Zenoh Discussion (GitHub Issue #512)](https://github.com/dora-rs/dora/issues/512)
- [Dora Autoware Localization Demo](https://github.com/dora-rs/dora-autoware-localization-demo)

```

```
