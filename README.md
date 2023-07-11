<p align="center">
    <img src="./docs/src/logo.svg" width="400">
</p>



<h2 align="center">
  <a href="https://dora.carsmos.ai">Website</a>
  |
  <a href="https://dora.carsmos.ai/docs/api/python-api">Python API</a>
  -
  <a href="https://docs.rs/dora-node-api/latest/dora_node_api/">Rust API</a>
  |
  <a href="https://dora.carsmos.ai/docs/guides/">Guide</a>
  |
  <a href="https://github.com/orgs/dora-rs/discussions">Discussion</a>
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

---

## Why dora-rs?

In 2023, AI is booming! Robotic framework however hasn't changed much in years... This is why we create dora-rs! dora-rs is a new robotic framework that brings modernity into robotic application.

dora-rs can already show impressive performance, up to 17x speedup compared to ROS2! This is the result of using our own shared memory server and Apache Arrow to achieve zero copy!

Those performance improvements make a world of difference for beginners, AI practitioners, and weekend hobbyists who have been limited by the lack of support for Python in this field!

And that's only one example of the many innovative features that we can show for dora-rs!

<p align="center">
    <img src="./docs/src/latency.png" width="600">
    
</p>

## Installation

Quickest way:

```bash
cargo install dora-cli
alias dora='dora-cli'
cargo install dora-coordinator
cargo install dora-daemon
pip install dora-rs ## For Python API

dora --help
```

For more installation guideline, check out our installation guide here: https://dora.carsmos.ai/docs/guides/Installation/installing

## Getting Started


1. Install the example python dependencies:
```bash
pip install opencv-python numpy pyarrow
```

2. Get some example operators:
```bash
wget https://raw.githubusercontent.com/dora-rs/dora/main/examples/python-operator-dataflow/webcam.py
wget https://raw.githubusercontent.com/dora-rs/dora/main/examples/python-operator-dataflow/plot.py
wget https://raw.githubusercontent.com/dora-rs/dora/main/examples/python-operator-dataflow/utils.py
wget https://raw.githubusercontent.com/dora-rs/dora/main/examples/python-operator-dataflow/dataflow.yml
```

3. Start the dataflow
```bash
dora start dataflow.yaml --attach --hot-reload
```

To go further, you can add a yolov5 operator, check out our getting started here: https://dora.carsmos.ai/docs/guides/getting-started/yolov5

## Documentation

The full documentation is available on our website: https://dora.carsmos.ai 

## Discussion

Our main communication channel is our Github Project Discussion page: https://github.com/orgs/dora-rs/discussions

Feel free to reach out on any topic, issues or ideas.

We also have [a contributing guide](CONTRIBUTING.md).

## License

This project is licensed under Apache-2.0. Check out [NOTICE.md](NOTICE.md) for more information.
