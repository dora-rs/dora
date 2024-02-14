# Dora Node API for C++

## Build 

- Clone the `dora` repository:
  ```bash
  > git clone https://github.com/dora-rs/dora.git
  > cd dora
  ```
- Build the `dora-node-api-cxx` package:
  ```bash
  cargo build --package dora-node-api-cxx
  ```
  - This will result in `dora-node-api.h` and `dora-node-api.cc` files in the `target/cxxbridge/dora-node-api-cxx` directory.
- Include the `dora-node-api.h` header file in your source file.
- Add the `dora-node-api.cc` file to your compile and link steps.

### Build with ROS2 Bridge

Dora features an experimental ROS2 Bridge that enables dora nodes to publish and subscribe to ROS2 topics.
To enable the bridge, use these steps:

- Clone the `dora` repository (see above).
- Source the ROS2 setup files (see [ROS2 docs](https://docs.ros.org/en/rolling/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html#source-the-setup-files))
- Optional: Source package-specific ROS2 setup files if you want to use custom package-specific ROS2 messages in the bridge (see [ROS2 docs](https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html#source-the-setup-file))
- Build the `dora-node-api-cxx` package **with the `ros2-bridge` feature enabled**:
  ```bash
  cargo build --package dora-node-api-cxx --feature ros2-bridge
  ```
  - In addition to the `dora-node-api.h` and `dora-node-api.cc` files, this will place a `dora-ros2-bindings.h` and a `dora-ros2-bindings.cc` file in the `target/cxxbridge/dora-node-api-cxx` directory.
- Include both the `dora-node-api.h` and the `dora-ros2-bindings.h` header files in your source file.
- Add the `dora-node-api.cc` and `dora-ros2-bindings.cc` files to your compile and link steps.

## Usage

### Init Dora Node

TODO

### Receiving Events

TODO

### Sending Outputs

TODO

## Using the ROS2 Bridge

### Initializing the ROS2 Context

TODO

### Creating Topics

TODO

### Publish

TODO

### Subscriptions

TODO
