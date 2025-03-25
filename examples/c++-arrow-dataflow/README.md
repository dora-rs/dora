# Dora C++ Dataflow Example 2

This example demonstrates how to exchange data between Dora's Rust-based runtime and C++ using Apache Arrow arrays. Through the event_as_arrow_input() and send_arrow_output() functions exposed in the dora-node-api.h header, your C++ nodes can efficiently receive and send structured data within the Dora dataflow system. These functions leverage Apache Arrow's memory-efficient serialization format, allowing data to move seamlessly across language boundaries.

## Required System Dependencies

- **Apache Arrow C++ Library**: Version 19.0.1 or later

Installing it should look like this:

### For Ubuntu

```bash
sudo apt-get update
sudo apt-get install -y -V ca-certificates lsb-release wget
wget https://apache.jfrog.io/artifactory/arrow/$(lsb_release --id --short | tr 'A-Z' 'a-z')/apache-arrow-apt-source-latest-$(lsb_release --codename --short).deb
sudo apt-get install -y -V ./apache-arrow-apt-source-latest-$(lsb_release --codename --short).deb
sudo apt-get update
sudo apt-get install -y -V libarrow-dev libarrow-glib-dev
```

### For macOS

```bash
brew update
brew install apache-arrow
fi
```

## Compile and Run

To try it out, you can use the [`run.rs`](./run.rs) binary. It performs all required build steps and then starts the dataflow. Use the following command to run it: `cargo run --example cxx-dataflow2`. For manual build, check build system for
`cxx-dataflow` example.
