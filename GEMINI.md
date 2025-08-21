# Gemini Code Assistant Context for `dora-rs`

This document provides context for the Gemini Code Assistant to understand the `dora-rs` project.

## Project Overview

`dora-rs` is a high-performance, dataflow-oriented framework for building robotics and AI applications. It is written primarily in Rust to ensure low-latency and real-time capabilities. The framework supports multiple programming languages through its APIs, including Rust, Python, C, and C++.

The core concept in `dora-rs` is the "dataflow," which is a graph of interconnected "nodes." Each node is a separate process that performs a specific task, such as reading from a sensor, running an AI model, or controlling an actuator. Nodes communicate with each other by sending and receiving messages, using shared memory for local communication and TCP for distributed setups.

The project includes a "Node Hub" which provides a collection of pre-built nodes for common tasks, simplifying the process of building complex applications.

## Building and Running

### Installation

The recommended way to install the `dora-rs` command-line interface (CLI) is using `pip`:

```bash
pip install dora-rs-cli
```

Alternatively, you can use the installation script:

```bash
curl --proto '=https' --tlsv1.2 -LsSf https://github.com/dora-rs/dora/releases/latest/download/dora-cli-installer.sh | sh
```

### Running a Dataflow

Applications in `dora-rs` are defined by a YAML file that describes the dataflow. To run a dataflow, use the `dora` CLI:

1.  **Start the `dora` daemon:**
    ```bash
    dora up
    ```

2.  **Build the dataflow dependencies:**
    ```bash
    dora build <path_to_dataflow.yml>
    ```

3.  **Run the dataflow:**
    ```bash
    dora start <path_to_dataflow.yml>
    ```

4.  **Stop the dataflow:**
    ```bash
    dora stop --all
    ```

### Building from Source

To build the project from source, you will need the Rust toolchain installed.

1.  **Clone the repository:**
    ```bash
    git clone https://github.com/dora-rs/dora.git
    cd dora
    ```

2.  **Build the project:**
    ```bash
    cargo build --release
    ```

## Development Conventions

*   **Code Formatting:** The project uses `rustfmt` for code formatting. Before submitting a pull request, please format your code with `cargo fmt --all`.
*   **Linting:** The project uses `clippy` for linting. Please run `cargo clippy --all` to check for any issues.
*   **Testing:** The project has a comprehensive test suite that is run on the CI. You can run the tests locally with `cargo test --all`.
*   **Contributions:** Contributions are welcome. Please read the `CONTRIBUTING.md` file for guidelines on how to contribute to the project.
*   **Issue Management:** The project uses a `dora-bot` to manage issue assignments. You can use `@dora-bot assign me` to assign an issue to yourself.
