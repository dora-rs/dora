# Dora C++ Dataflow Example

This example shows usage of event_as_arrow_input() and send_arrow_output(). We can send and receive arrow arrays using these functions which can be serialized and deserialized on either files easily. These functions are implemented in rust and are provided through dora-node-api.h. Currently this requires to have arrow installed on user system as required during build process.  

## Compile and Run

To try it out, you can use the [`run.rs`](./run.rs) binary. It performs all required build steps and then starts the dataflow. Use the following command to run it: `cargo run --example cxx-dataflow2`.

**Build the dora coordinator and runtime:**

- Build the `dora-coordinator` executable using `cargo build -p dora-coordinator --release`
- Build the `dora-runtime` executable using `cargo build -p dora-runtime --release`

**Run the dataflow:**

- Start the `dora-coordinator`, passing the paths to the dataflow file and the `dora-runtime` as arguments:

  ```
  ../../target/release/dora-daemon --run-dataflow dataflow.yml ../../target/release/dora-runtime
  ```
