# Dora C++ Dataflow Example 2

This example demonstrates how to exchange data between Dora's Rust-based runtime and C++ using Apache Arrow arrays. Through the `event_as_arrow_input()` and `send_arrow_output()` functions exposed in the `dora-node-api.h` header, your C++ nodes can efficiently receive and send structured data within the Dora dataflow system. These functions leverage Apache Arrow's memory-efficient serialization format, allowing data to move seamlessly across language boundaries.

## Attaching Metadata To Outputs

The node API exposes a reusable `Metadata` object that you can populate from C++ before sending Arrow arrays. Create a fresh instance with `new_metadata()` and set typed parameters via helpers such as `set_bool`, `set_int`, `set_list_float`, or `set_list_string`. Pass the resulting pointer to `send_arrow_output` to include the metadata with your message:

```cpp
auto metadata = new_metadata();
metadata->set_string("source", "cpp-node");
metadata->set_int("iteration", counter);

rust::Vec<int64_t> checkpoints;
checkpoints.push_back(counter);
checkpoints.push_back(counter * 2);
metadata->set_list_int("checkpoints", std::move(checkpoints));

auto result = send_arrow_output(
    node.send_output,
    "counter",
    reinterpret_cast<uint8_t*>(&out_array),
    reinterpret_cast<uint8_t*>(&out_schema),
    std::move(metadata)
);
```

If you do not need to send metadata, call the four-argument overload without metadata.

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

To try it out, you can use the [`run.rs`](./run.rs) binary. It performs all required build steps and then starts the dataflow. Use the following command to run it: `cargo run --example cxx-arow-dataflow`. For manual build, check build system for
`cxx-dataflow` example.
