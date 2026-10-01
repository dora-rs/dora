# Dora C++ Dataflow Example

This example shows how to create dora operators and custom nodes with C++.

Dora provides a C++ API (`dora-node-api-cxx` / `dora-operator-api-cxx`, documented in the [C++ API reference](../../docs/api-cxx.md)); it is bridged from dora's Rust API via the `cxx` crate. It also provides a C API. The `node-rust-api` and `operator-rust-api` folders use the C++ API, while the `node-c-api` and `operator-c-api` folders use the C API. Both approaches work, so you can choose the API that fits your application better.

## Compile and Run

To try it out, you can use the [`run.rs`](./run.rs) binary. It performs all required build steps and then starts the dataflow. Use the following command to run it: `cargo run --example cxx-dataflow`.

For a manual build, follow these steps:

- Create a `build` folder in this directory
- Build the C++ variants (`node-rust-api`, `operator-rust-api`), which are bridged from the Rust API with the `cxx` crate:
  ```
  cargo build -p dora-node-api-cxx
  cargo build -p dora-operator-api-cxx
  ```
  This only builds the crates. Producing the `build/node_rust_api` and `build/operator_rust_api` artifacts that `dataflow.yml` expects additionally requires copying the generated bridge sources (`target/cxxbridge/dora-node-api-cxx/src/lib.rs.{cc,h}` and the `dora-operator-api-cxx` equivalent) into `build/`, writing the `build/operator.h` shim, compiling the `node-rust-api` / `operator-rust-api` sources against them (linking `-l dora_node_api_cxx` and `-l dora_operator_api_cxx -L target/debug`), and building the operator as a shared library. [`run.rs`](./run.rs) does all of this, so running it is the simplest way to build the C++ variants.
- The steps below build only the C-API variants (`node_c_api`, `operator_c_api`). `dataflow.yml` also needs the C++ artifacts above, so the C-API half alone is not enough to run the example end to end.
- Compile the `dora-node-api-c` crate into a static library.
  - Run `cargo build -p dora-node-api-c --release`
  - The resulting staticlib is then available under `../../target/release/libdora-node-api-c.a`.
- Compile the `node-c-api/main.cc` (e.g. using `clang++`) and link the staticlib
  - For example, use the following command:
    ```
    clang++ node-c-api/main.cc <FLAGS> -std=c++14 -ldora_node_api_c -L ../../target/release --output build/node_c_api
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

**Build the dora CLI:**

- Build the `dora` executable using `cargo build -p dora-cli --release`
  - This is the only dora binary you need: it embeds the coordinator and the
    daemon, and hosts shared-library operators like the one above through its
    `dora runtime` subcommand, which the daemon spawns for you.

**Run the dataflow:**

- Run the dataflow with the CLI built above:

  ```
  ../../target/release/dora run dataflow.yml
  ```
