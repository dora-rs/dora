#

<p align="center">
    <img src="https://raw.githubusercontent.com/dora-rs/dora/main/docs/src/logo.svg" width="400"/>
</p>

<h2 align="center">
  <a href="https://www.dora-rs.ai">官网</a>
  |
  <a href="https://dora-rs.ai/docs/guides/getting-started/conversation_py/">Python API</a>
  |
  <a href="https://docs.rs/dora-node-api/latest/dora_node_api/">Rust API</a>
  |
  <a href="https://www.dora-rs.ai/docs/guides/">指南</a>
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
| **操作系统**                            | Linux: Arm 32 ⭐✅ Arm 64 ⭐✅ x64_86 ⭐✅ <br>MacOS: Arm 64 ⭐✅ x64_86 ✅<br>Windows: x64_86 🆗<br> Android: 🛠️ (Blocked by: https://github.com/elast0ny/shared_memory/issues/32) <br> IOS: 🛠️ |
| **消息格式**                | Arrow ✅ <br> 标准规范 🛠️                                                                                                                                                          |
| **本地通信模块**           | 共享内存 ✅ <br> [Cuda IPC](https://arrow.apache.org/docs/python/api/cuda.html) 📐                                                                                                          |
| **远程通信模块**          | [Zenoh](https://zenoh.io/) 📐                                                                                                                                                                    |
| **指标、追踪与日志** | Opentelemetry 📐                                                                                                                                                                                 |
| **配置**                 | YAML ✅                                                                                                                                                                                          |
| **包管理**               | [pip](https://pypi.org/): Python 节点 ✅ Rust 节点 ✅ C/C++ 节点 🛠️ <br>[cargo](https://crates.io/): Rust 节点 ✅                                                                                |

> - ⭐ = 推荐方案
> - ✅ = 一级支持（官方全面支持）
> - 🆗 = 尽力支持（社区主导支持）
> - 📐 = 实验性功能（期待贡献）
> - 🛠️ = 暂不支持（期待社区贡献实现）
>
> 开放共建，欢迎来PR！ 🙋

## Node Hub

> 欢迎自由修改本README文档，添加您开发的节点说明，让整个社区共同受益！

| Type                          | Title                                                                                               | Support             | Description                                      | Downloads                                                                     | License                                                                    |
| ----------------------------- | --------------------------------------------------------------------------------------------------- | ------------------- | ------------------------------------------------ | ----------------------------------------------------------------------------- | -------------------------------------------------------------------------- |
| 相机                        | [PyOrbbeckSDK](https://github.com/dora-rs/dora/blob/main/node-hub/dora-pyorbbecksdk)                | 📐                  | 奥比中光深度相机              | ![Downloads](https://img.shields.io/pypi/dm/dora-pyorbbecksdk?label=%20)      | ![License](https://img.shields.io/pypi/l/dora-pyorbbecksdk?label=%20)      |
| 相机                        | [PyRealsense](https://github.com/dora-rs/dora/blob/main/node-hub/dora-pyrealsense)                  | Linux🆗 <br> Mac🛠️  | Realsense深度相机                   | ![Downloads](https://img.shields.io/pypi/dm/dora-pyrealsense?label=%20)       | ![License](https://img.shields.io/pypi/l/dora-pyrealsense?label=%20)       |
| 相机                        | [OpenCV Video Capture](https://github.com/dora-rs/dora/blob/main/node-hub/opencv-video-capture)     | ✅                  | OpenCV 相机的图像流                  | ![Downloads](https://img.shields.io/pypi/dm/opencv-video-capture?label=%20)   | ![License](https://img.shields.io/pypi/l/opencv-video-capture?label=%20)   |
| 外围设备                    | [Keyboard](https://github.com/dora-rs/dora/blob/main/node-hub/dora-keyboard)                        | ✅                  | 键盘监听                           | ![Downloads](https://img.shields.io/pypi/dm/dora-keyboard?label=%20)          | ![License](https://img.shields.io/pypi/l/dora-keyboard?label=%20)          |
| 外围设备                    | [Microphone](https://github.com/dora-rs/dora/blob/main/node-hub/dora-microphone)                    | ✅                  | 麦克风🎤                            | ![Downloads](https://img.shields.io/pypi/dm/dora-microphone?label=%20)        | ![License](https://img.shields.io/pypi/l/dora-microphone?label=%20)        |
| 外围设备                    | [PyAudio(Speaker)](https://github.com/dora-rs/dora/blob/main/node-hub/dora-pyaudio)                 | ✅                  | 扬声器🔉                        | ![Downloads](https://img.shields.io/pypi/dm/dora-pyaudio?label=%20)           | ![License](https://img.shields.io/pypi/l/dora-pyaudio?label=%20)           |
| 执行器                      | [Feetech](https://github.com/dora-rs/dora-lerobot/blob/main/node-hub/feetech-client)                | 📐                  | Feetech                                    |                                                                               |                                                                            |
| 执行器                      | [Dynamixel](https://github.com/dora-rs/dora-lerobot/blob/main/node-hub/dynamixel-client)            | 📐                  | Dynamixel                                  |                                                                               |                                                                            |
| 底盘                       | [Agilex - UGV](https://github.com/dora-rs/dora/blob/main/node-hub/dora-ugv)                         | 🆗                  | Robomaster                               | ![Downloads](https://img.shields.io/pypi/dm/dora-ugv?label=%20)               | ![License](https://img.shields.io/pypi/l/dora-ugv?label=%20)               |
| 底盘                       | [DJI - Robomaster S1](https://huggingface.co/datasets/dora-rs/dora-robomaster)                      | 📐                  | Robomaster                                |                                                                               |                                                                            |
| 底盘                       | [Dora Kit Car](https://github.com/dora-rs/dora/blob/main/node-hub/dora-kit-car)                     | 🆗                  | 开源 Dora kit 底盘                              | ![Downloads](https://img.shields.io/pypi/dm/dora-kit-car?label=%20)           | ![License](https://img.shields.io/pypi/l/dora-kit-car?label=%20)           |
| 机械臂                           | [Alex Koch - Low Cost Robot](https://github.com/dora-rs/dora-lerobot/blob/main/robots/alexk-lcr)    | 📐                  | Alex Koch - 低成本机器人                |                                                                               |                                                                            |
| 机械臂                           | [Lebai - LM3](https://github.com/dora-rs/dora-lerobot/blob/main/node-hub/lebai-client)              | 📐                  | 乐白机械臂                                     |                                                                               |                                                                            |
| 机械臂                           | [Agilex - Piper](https://github.com/dora-rs/dora/blob/main/node-hub/dora-piper)                     | 🆗                  | Agilex 机械臂                                | ![Downloads](https://img.shields.io/pypi/dm/dora-piper?label=%20)             | ![License](https://img.shields.io/pypi/l/dora-piper?label=%20)             |
| 机器人                         | [Pollen - Reachy 1](https://github.com/dora-rs/dora-lerobot/blob/main/node-hub/dora-reachy1)        | 📐                  | Reachy 1                                   |                                                                               |                                                                            |
| 机器人                         | [Pollen - Reachy 2](https://github.com/dora-rs/dora/blob/main/node-hub/dora-reachy2)                | 🆗                  | Reachy 2                                  | ![Downloads](https://img.shields.io/pypi/dm/dora-reachy2?label=%20)           | ![License](https://img.shields.io/pypi/l/dora-reachy2?label=%20)           |
| 机器人                         | [Trossen - Aloha](https://github.com/dora-rs/dora-lerobot/blob/main/robots/aloha)                   | 📐                  | Aloha                                      |                                                                               |                                                                            |
| 语音活动检测(VAD) | [Silero VAD](https://github.com/dora-rs/dora/blob/main/node-hub/dora-vad)                           | ✅                  | Silero 语音活动检测                  | ![Downloads](https://img.shields.io/pypi/dm/dora-vad?label=%20)               | ![License](https://img.shields.io/pypi/l/dora-vad?label=%20)               |
| 语音转文本(STT)           | [Whisper](https://github.com/dora-rs/dora/blob/main/node-hub/dora-distil-whisper)                   | ✅                  | 将音频转录为文本                         | ![Downloads](https://img.shields.io/pypi/dm/dora-distil-whisper?label=%20)    | ![License](https://img.shields.io/pypi/l/dora-distil-whisper?label=%20)    |
| 目标检测              | [Yolov8](https://github.com/dora-rs/dora/blob/main/node-hub/dora-yolo)                              | ✅                  | 目标检测                                 | ![Downloads](https://img.shields.io/pypi/dm/dora-yolo?label=%20)              | ![License](https://img.shields.io/pypi/l/dora-yolo?label=%20)              |
| 分割                  | [SAM2](https://github.com/dora-rs/dora/blob/main/node-hub/dora-sam2)                                | Cuda✅ <br> Metal🛠️ | 分割万物                                 | ![Downloads](https://img.shields.io/pypi/dm/dora-sam2?label=%20)              | ![License](https://img.shields.io/pypi/l/dora-sam2?label=%20)              |
| 大语言模型(LLM)     | [Qwen2.5](https://github.com/dora-rs/dora/blob/main/node-hub/dora-qwen)                             | ✅                  | 千问大语言模型                  | ![Downloads](https://img.shields.io/pypi/dm/dora-qwen?label=%20)              | ![License](https://img.shields.io/pypi/l/dora-qwen?label=%20)              |
| 视觉语言模型(VLM)    | [Qwen2.5-vl](https://github.com/dora-rs/dora/blob/main/node-hub/dora-qwen2-5-vl)                    | ✅                  | 千问2.5多模态模型           | ![Downloads](https://img.shields.io/pypi/dm/dora-qwen2-5-vl?label=%20)        | ![License](https://img.shields.io/pypi/l/dora-qwen2-5-vl?label=%20)        |
| 视觉语言模型(VLM)    | [InternVL](https://github.com/dora-rs/dora/blob/main/node-hub/dora-internvl)                        | 🆗                  | InternVL 视觉语言模型              | ![Downloads](https://img.shields.io/pypi/dm/dora-internvl?label=%20)          | ![License](https://img.shields.io/pypi/l/dora-internvl?label=%20)          |
| 视觉语言行为(VLA)   | [RDT-1B](https://github.com/dora-rs/dora/blob/main/node-hub/dora-rdt-1b)                            | 🆗                  | 基于Robotic Diffusion Transformer的策略推理系统 | ![Downloads](https://img.shields.io/pypi/dm/dora-rdt-1b?label=%20)            | ![License](https://img.shields.io/pypi/l/dora-rdt-1b?label=%20)            |
| 翻译                   | [ArgosTranslate](https://github.com/dora-rs/dora/blob/main/node-hub/dora-argotranslate)             | 🆗                  | 开源的翻译引擎                   | ![Downloads](https://img.shields.io/pypi/dm/dora-argotranslate?label=%20)     | ![License](https://img.shields.io/pypi/l/dora-argotranslate?label=%20)     |
| 翻译                   | [Opus MT](https://github.com/dora-rs/dora/blob/main/node-hub/dora-opus)                             | 🆗                  | 实时多语言文本互译                  | ![Downloads](https://img.shields.io/pypi/dm/dora-opus?label=%20)              | ![License](https://img.shields.io/pypi/l/dora-opus?label=%20)              |
| 文本转语音(TTS)           | [Kokoro TTS](https://github.com/dora-rs/dora/blob/main/node-hub/dora-kokoro-tts)                    | ✅                  | 高效文本转语音（TTS）                         | ![Downloads](https://img.shields.io/pypi/dm/dora-kokoro-tts?label=%20)        | ![License](https://img.shields.io/pypi/l/dora-kokoro-tts?label=%20)        |
| 记录                     | [Llama Factory Recorder](https://github.com/dora-rs/dora/blob/main/node-hub/llama-factory-recorder) | 🆗                  | 大语言模型(LLM)与视觉语言模型(VLM)训练数据采集方案                | ![Downloads](https://img.shields.io/pypi/dm/llama-factory-recorder?label=%20) | ![License](https://img.shields.io/pypi/l/llama-factory-recorder?label=%20) |
| 记录                     | [LeRobot Recorder](https://github.com/dora-rs/dora-lerobot/blob/main/node-hub/lerobot-dashboard)    | 📐                  | LeRobot 数据采集助手                          |                                                                               |                                                                            |
| 可视化                 | [Plot](https://github.com/dora-rs/dora/blob/main/node-hub/opencv-plot)                              | ✅                  | 简易OpenCV可视化工具                 | ![Downloads](https://img.shields.io/pypi/dm/dora-yolo?label=%20)              | ![License](https://img.shields.io/pypi/l/opencv-plot?label=%20)            |
| 可视化                 | [Rerun](https://github.com/dora-rs/dora/blob/main/node-hub/dora-rerun)                              | ✅                  | 可视化工具                               | ![Downloads](https://img.shields.io/pypi/dm/dora-rerun?label=%20)             | ![License](https://img.shields.io/pypi/l/dora-rerun?label=%20)             |
| 仿真                     | [Mujoco](https://github.com/dora-rs/dora-lerobot/blob/main/node-hub/mujoco-client)                  | 📐                  | Mujoco 仿真                                 |                                                                               |                                                                            |
| 仿真                     | [Carla](https://github.com/dora-rs/dora-drives)                                                     | 📐                  | Carla 仿真                                  |                                                                               |                                                                            |
| 仿真                     | [Gymnasium](https://github.com/dora-rs/dora-lerobot/blob/main/gym_dora)                             | 📐                  | 实验性 OpenAI Gymnasium 桥接模块             |                                                                               |                                                                            |

## Examples

| Type           | Title                                                                                                        | Description                                         | Last Commit                                                                                                        |
| -------------- | ------------------------------------------------------------------------------------------------------------ | --------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------ |
| 语音          | [Speech to Text(STT)](https://github.com/dora-rs/dora/blob/main/examples/speech-to-text)                     | 将语音转换为文本                           | ![License](https://img.shields.io/github/last-commit/dora-rs/dora?path=examples%2Fspeech-to-text&label=%20)        |
| 语音          | [Translation](https://github.com/dora-rs/dora/blob/main/examples/translation)                                | 实时翻译音频                       | ![License](https://img.shields.io/github/last-commit/dora-rs/dora?path=examples%2Ftranslation&label=%20)           |
| 视觉         | [Vision Language Model(VLM)](https://github.com/dora-rs/dora/blob/main/examples/vlm)                         | 使用视觉语言模型（VLM）来理解图像                     | ![License](https://img.shields.io/github/last-commit/dora-rs/dora?path=examples%2Fvlm&label=%20)                   |
| 视觉         | [YOLO](https://github.com/dora-rs/dora/blob/main/examples/python-dataflow)                                   | 使用 YOLO 在图像中检测物体             | ![License](https://img.shields.io/github/last-commit/dora-rs/dora?path=examples%2Fpython-dataflow&label=%20)       |
| 视觉         | [Camera](https://github.com/dora-rs/dora/blob/main/examples/camera)                                          | 简单的网络摄像头图像绘制示例                          | ![License](https://img.shields.io/github/last-commit/dora-rs/dora?path=examples%2Fcamera&label=%20)                |
| 模型训练 | [Piper RDT](https://github.com/dora-rs/dora/blob/main/examples/piper)                                        | Piper RDT 数据处理流水线                                  | ![License](https://img.shields.io/github/last-commit/dora-rs/dora?path=examples%2Fpiper&label=%20)                 |
| 模型训练 | [LeRobot - Alexander Koch](https://raw.githubusercontent.com/dora-rs/dora-lerobot/refs/heads/main/README.md) | 使用 LeRobot 来训练 Alexander Koc 低成本机器人 | ![License](https://img.shields.io/github/last-commit/dora-rs/dora-lerobot?path=robots&label=%20)                   |
| ROS2           | [C++ ROS2 Example](https://github.com/dora-rs/dora/blob/main/examples/c++-ros2-dataflow)                     | 使用 C++ 开发的 ROS2 示例                              | ![License](https://img.shields.io/github/last-commit/dora-rs/dora?path=examples%2Fc%2b%2b-ros2-dataflow&label=%20) |
| ROS2           | [Rust ROS2 Example](https://github.com/dora-rs/dora/blob/main/examples/rust-ros2-dataflow)                   | 使用 Rust 开发的 ROS 2 示例                             | ![License](https://img.shields.io/github/last-commit/dora-rs/dora?path=examples%2Frust-ros2-dataflow&label=%20)    |
| ROS2           | [Python ROS2 Example](https://github.com/dora-rs/dora/blob/main/examples/python-ros2-dataflow)               | 使用 Python 开发的 ROS 2 示例                           | ![License](https://img.shields.io/github/last-commit/dora-rs/dora?path=examples%2Fpython-ros2-dataflow&label=%20)  |
| 基准      | [GPU Benchmark](https://github.com/dora-rs/dora/blob/main/examples/cuda-benchmark)                           | dora-rs 的 GPU 性能基准测试                            | ![License](https://img.shields.io/github/last-commit/dora-rs/dora?path=examples%2Fcuda-benchmark&label=%20)        |
| 基准      | [CPU Benchmark](https://github.com/dora-rs/dora-benchmark/blob/main)                                         | dora-rs 的 CPU 性能基准测试                            | ![License](https://img.shields.io/github/last-commit/dora-rs/dora-benchmark?path=dora-rs&label=%20)                |
| 指南       | [Rust Example](https://github.com/dora-rs/dora/blob/main/examples/rust-dataflow)                             | Rust示例                                  | ![License](https://img.shields.io/github/last-commit/dora-rs/dora?path=examples%2Frust-dataflow&label=%20)         |
| 指南       | [Python Example](https://github.com/dora-rs/dora/blob/main/examples/python-dataflow)                         | Python示例                                | ![License](https://img.shields.io/github/last-commit/dora-rs/dora?path=examples%2Fpython-dataflow&label=%20)       |
| 指南       | [CMake Example](https://github.com/dora-rs/dora/blob/main/examples/cmake-dataflow)                           | CMake示例                                 | ![License](https://img.shields.io/github/last-commit/dora-rs/dora?path=examples%2Fcmake-dataflow&label=%20)        |
| 指南       | [C Example](https://github.com/dora-rs/dora/blob/main/examples/c-dataflow)                                   | C节点示例                                 | ![License](https://img.shields.io/github/last-commit/dora-rs/dora?path=examples%2Fc-dataflow&label=%20)            |
| 指南       | [CUDA Example](https://github.com/dora-rs/dora/blob/main/examples/cuda-benchmark)                            | CUDA 零拷贝示例                        | ![License](https://img.shields.io/github/last-commit/dora-rs/dora?path=examples%2Fcuda-benchmark&label=%20)        |
| 指南       | [C++ Example](https://github.com/dora-rs/dora/blob/main/examples/c++-dataflow)                               | C++ 节点示例                               | ![License](https://img.shields.io/github/last-commit/dora-rs/dora?path=examples%2Fc%2b%2b-dataflow&label=%20)      |

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

- 输出示例:

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

- 通过上例可知：摄像头将图像数据同时传输至以下两个终端：1、 Rerun 可视化平台 - 实时显示原始视频流。 2、YOLO 模型 - 生成目标检测框数据后，回传至 Rerun 进行可视化渲染

### 文档

完整文档详见 [官网](https://dora-rs.ai/)。
官网上的 [指引](https://dora-rs.ai/docs/guides/) 提供丰富指南文档

## Dora 是什么？ Dora 具备哪些功能

**D**ataflow-**O**riented **R**obotic **A**rchitecture (`dora-rs`) 是一个能让机器人应用程序的创建变得快速且简单的框架。

`dora-rs` 实现了一种声明式的数据流范式，在这种范式中，任务被划分到作为独立进程而隔离的节点之间。

这种数据流范式的优势在于创建了一个抽象层，使得机器人应用程序具有模块化特点，并且易于配置。 

### TCP通信与共享内存

节点之间的通信在同一台机器上通过共享内存来处理，而在分布式机器上则通过TCP（传输控制协议）来处理。我们的共享内存实现会跟踪跨进程的消息，并在消息过时的时候将其丢弃。共享内存插槽会被缓存起来，以避免进行新的内存分配。 

### Arrow 息格式

节点之间采用 Apache Arrow 数据格式进行通信。

[Apache Arrow](https://github.com/apache/arrow-rs) 是一种适用于扁平化和层级化数据的通用内存格式。该内存格式支持零拷贝读取，无需序列化开销即可实现闪电般快速的数据访问。其定义的C数据接口不要求任何构建时或链接时依赖，这意味着除您所用编程语言的原生编译器外，`dora-rs` 无需额外的编译步骤。


### 可观测性

dora-rs 使用 Opentelemetry 来记录您所有的日志、指标和追踪信息。这意味着数据和遥测信息可以通过一个共享的抽象进行链接。

[Opentelemetry](https://opentelemetry.io/) 是一个开源的可观测性标准，使得 dora-rs 的遥测数据可以被大多数后端收集，例如 Elasticsearch、Prometheus、Datadog 等。

Opentelemetry 具有语言无关、后端无关的特性，并且能够轻松收集分布式数据，非常适合 dora-rs 应用程序。

### ROS2 桥接 (ROS2 Bridge)

**注意:** 此功能标记为不稳定 (unstable)

- 免编译消息传递到 ROS 2
- ROS 2 消息 <-> Arrow 数组自动转换。

```python
import pyarrow as pa

# 配置样板代码...
turtle_twist_writer = ...

## 基于 Arrow 的 ROS2 Twist 消息
## 无需导入 ROS2
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

> 您可以借助 ChatGPT 自动生成 Arrow 数据格式代码: https://chat.openai.com/share/4eec1c6d-dbd2-46dc-b6cd-310d2895ba15

## 使用 Zenoh 的分布式数据流 (Distributed Dataflow with Zenoh)

**dora-rs** 支持使用 Zenoh（一种高性能的发布/订阅和查询协议）进行分布式数据流，以解决跨多台机器运行数据流的需求 。

### 什么是 Zenoh?

- **定义:**  
  [Zenoh](https://zenoh.io) 是一款开源的通信中间件，提供发布 / 订阅以及查询功能。
- **在 DORA 中的优势:**
  - 简化分布式节点之间的通信；
  - 处理网络地址转换（NAT）遍历以及网络间通信；
  - 与 DORA 集成，在本地通信仍使用高效共享内存的同时，管理远程数据交换；

### 开启 Zenoh 支持

1. **运行 Zenoh 路由 (`zenohd`):**  
   启动一个 Zenoh 守护进程来协调通信。例如，使用 Docker 来操作:

   ```bash
   docker run -p 7447:7447 -p 8000:8000 --name zenoh-router eclipse/zenohd:latest
   ```

````markdown
## 创建一个 Zenoh 配置文件 🎛️

创建一个包含路由器端点详细信息的 Zenoh 配置文件(例如 `zenoh.json5`)：

```json5
{
  connect: {
    endpoints: ["tcp/203.0.113.10:7447"],
  },
}
```
````

---

## Zenoh 开启后，启用 DORA 守候进程 🚀

在每台机器上，导出配置并启动守护进程：

```bash
export ZENOH_CONFIG=/path/to/zenoh.json5
dora daemon --coordinator-addr <COORD_IP> --machine-id <MACHINE_NAME>
```

---

## 通过 YAML 部署分布式节点 📄

使用 _unstable_deploy 键标记要进行远程部署的节点：

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

## 启动 Coordinator 和 Dataflow 🏁

在指定的机器上运行 `coordinator` 并启动 `dataflow` ：

```bash
dora coordinator
dora start dataflow.yml
```

---

## 分布式数据流的 YAML 示例 📘

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

## 贡献

我们热衷于支持各种经验水平的贡献者，并且非常希望看到您参与到这个项目中来。请查看 [贡献指南](https://github.com/dora-rs/dora/blob/main/CONTRIBUTING.md) 以着手参与。


## 讨论

我们主要的沟通渠道如下: 

- [Our Discord server](https://discord.gg/6eMGGutkfE)
- [Our Github Project Discussion](https://github.com/orgs/dora-rs/discussions)
- [Dora中文社区](https://doracc.com)

可以就任何话题、问题或想法与我们联系。

我们还有一份[贡献指南](CONTRIBUTING.md)。

## 许可证

该项目采用 Apache-2.0 许可证授权。 查看 [NOTICE.md](NOTICE.md) 文件了解详情。

---

## 进一步资源  📚

- [Zenoh 文档](https://zenoh.io/docs/)
- [DORA Zenoh Discussion (GitHub Issue #512)](https://github.com/dora-rs/dora/issues/512)
- [Dora Autoware 定位演示](https://github.com/dora-rs/dora-autoware-localization-demo)
