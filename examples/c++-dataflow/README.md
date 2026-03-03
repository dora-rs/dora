# Adora C++ Dataflow Example

This example shows how to create adora operators and custom nodes with C++.

Adora does not provide a C++ API yet, but we can create adapters for either the C or Rust API. The `operator-rust-api` and `node-rust-api` folders implement an example operator and node based on adora's Rust API, using the `cxx` crate for bridging. The `operator-c-api` and `node-c-api` show how to create operators and nodes based on adora's C API. Both approaches work, so you can choose the API that fits your application better.

## Compile and Run

To try it out, you can use the [`run.rs`](./run.rs) binary. It performs all required build steps and then starts the dataflow. Use the following command to run it: `cargo run --example cxx-dataflow`.

For a manual build, follow these steps:

- Create a `build` folder in this directory
- Build the `cxx-dataflow-example-node-rust-api` and `cxx-dataflow-example-operator-rust-api` crates:
  ```
  cargo build -p cxx-dataflow-example-node-rust-api --release
  cargo build -p cxx-dataflow-example-operator-rust-api --release
  ```
- Compile the `adora-node-api-c` crate into a static library.
  - Run `cargo build -p adora-node-api-c --release`
  - The resulting staticlib is then available under `../../target/release/libadora-node-api-c.a`.
- Compile the `node-c-api/main.cc` (e.g. using `clang++`) and link the staticlib
  - For example, use the following command:
    ```
    clang++ node-c-api/main.cc <FLAGS> -std=c++14 -ladora_node_api_c -L ../../target/release --output build/node_c_api
    ```
  - The `<FLAGS>` depend on the operating system and the libraries that the C node uses. The following flags are required for each OS:
    - Linux: `-lm -lrt -ldl -pthread`
    - macOS: `-framework CoreServices -framework Security -l System -l resolv -l pthread -l c -l m`
    - Windows:
      ```
      -ladvapi32 -luserenv -lkernel32 -lws2_32 -lbcrypt -lncrypt -lschannel -lntdll -liphlpapi
      -lcfgmgr32 -lcredui -lcrypt32 -lcryptnet -lfwpuclnt -lgdi32 -lmsimg32 -lmswsock -lole32
      -loleaut32 -lopengl32 -lsecur32 -lshell32 -lsynchronization -luser32 -lwinspool
      -Wl,-nodefaultlib:libcmt -D_DLL -lmsvcrt
      ```
      Also: On Windows, the output file should have an `.exe` extension: `--output build/c_node.exe`
- Compile the `operator-c-api/operator.cc` file into a shared library.
  - For example, use the following commands:
    ```
    clang++ -c operator-c-api/operator.cc -std=c++14 -o build/operator_c_api.o -fPIC
    clang++ -shared build/operator_c_api.o -o build/liboperator_c_api.so
    ```
    Omit the `-fPIC` argument on Windows. Replace the `liboperator_c_api.so` name with the shared library standard library prefix/extensions used on your OS, e.g. `.dll` on Windows.

**Build the adora coordinator and runtime:**

- Build the `adora-coordinator` executable using `cargo build -p adora-coordinator --release`
- Build the `adora-runtime` executable using `cargo build -p adora-runtime --release`

**Run the dataflow:**

- Start the `adora-coordinator`, passing the paths to the dataflow file and the `adora-runtime` as arguments:

  ```
  ../../target/release/adora-daemon --run-dataflow dataflow.yml ../../target/release/adora-runtime
  ```
