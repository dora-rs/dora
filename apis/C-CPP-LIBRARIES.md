# Using Pre-built Dora C/C++ Libraries

Pre-built C and C++ static libraries are published as GitHub release artifacts for each Dora release.

## Available Libraries

### C API
- **dora-node-api-c** - C API for creating Dora nodes
- **dora-operator-api-c** - C API for creating Dora operators

### C++ API
- **dora-node-api-cxx** - C++ API for creating Dora nodes (cxx bridge)
- **dora-operator-api-cxx** - C++ API for creating Dora operators (cxx bridge)

## Supported Targets

| Target Triple | Platform | Archive |
|---|---|---|
| `x86_64-unknown-linux-gnu` | Linux x86_64 (glibc) | `.tar.gz` |
| `aarch64-unknown-linux-gnu` | Linux ARM64 (glibc) | `.tar.gz` |
| `aarch64-apple-darwin` | macOS Apple Silicon | `.tar.gz` |
| `x86_64-pc-windows-msvc` | Windows x86_64 | `.zip` |

Intel macOS (`x86_64-apple-darwin`) is not published as a pre-built artifact — build from source with `cargo build --release -p dora-node-api-c -p dora-operator-api-c -p dora-node-api-cxx -p dora-operator-api-cxx`.

## Obtaining the API Libraries

There are two ways to obtain the C/C++ API libraries:

### Method 1: Download Pre-built Archives

Download the pre-built `.tar.gz` (Unix) or `.zip` (Windows) archives from the [Dora releases page](https://github.com/dora-rs/dora/releases). This is the recommended approach for most users.

```bash
# Download and extract C API for Linux x86_64
tar xzf dora-c-libraries-x86_64-unknown-linux-gnu.tar.gz
```

See [Download](#download) for the full list of available targets.

### Method 2: Build with `xtask`

Build from source and use the `xtask` tool to stage artifacts. Use this when building for an unsupported target (e.g., Intel macOS) or when you need a specific commit.

```bash
git clone https://github.com/dora-rs/dora.git
cd dora

# Build all C/C++ libraries
cargo build --release \
    -p dora-node-api-c \
    -p dora-operator-api-c \
    -p dora-node-api-cxx \
    -p dora-operator-api-cxx

# Stage to prefix directory (produces the same structure as the download archives)
TARGET=$(rustc -vV | sed -n 's/host: //p')

cargo run -p xtask -- stage dora-node-api-c \
    target/release "dora-c-libraries-$TARGET"
cargo run -p xtask -- stage dora-operator-api-c \
    target/release "dora-c-libraries-$TARGET"
cargo run -p xtask -- stage dora-node-api-cxx \
    target/release "dora-cpp-libraries-$TARGET"
cargo run -p xtask -- stage dora-operator-api-cxx \
    target/release "dora-cpp-libraries-$TARGET"
```

Both methods produce the same directory structure, so the CMake integration examples below work identically regardless of how you obtained the libraries.

## Download

1. Go to the [Dora releases page](https://github.com/dora-rs/dora/releases)
2. Download the archive for your target:
   - C API: `dora-c-libraries-<target>.tar.gz` (or `.zip` on Windows)
   - C++ API: `dora-cpp-libraries-<target>.tar.gz` (or `.zip` on Windows)

```bash
# Example: download and extract C++ libraries for Linux x86_64
tar xzf dora-cpp-libraries-x86_64-unknown-linux-gnu.tar.gz
```

## Directory Structure

**C libraries:**
```
dora-c-libraries-<target>/
├── include/
│   ├── node_api.h
│   ├── operator_api.h
│   └── operator_types.h
└── lib/
    ├── cmake/
    │   ├── dora-node-api-c/
    │   │   ├── dora-node-api-cConfig.cmake
    │   │   └── dora-node-api-cConfigVersion.cmake
    │   └── dora-operator-api-c/
    │       ├── dora-operator-api-cConfig.cmake
    │       └── dora-operator-api-cConfigVersion.cmake
    ├── libdora_node_api_c.a
    └── libdora_operator_api_c.a
```

**C++ libraries:**
```
dora-cpp-libraries-<target>/
├── include/
│   ├── dora-node-api.h
│   └── dora-operator-api.h
└── lib/
    ├── cmake/
    │   ├── dora-node-api-cxx/
    │   │   ├── dora-node-api-cxxConfig.cmake
    │   │   └── dora-node-api-cxxConfigVersion.cmake
    │   └── dora-operator-api-cxx/
    │       ├── dora-operator-api-cxxConfig.cmake
    │       └── dora-operator-api-cxxConfigVersion.cmake
    ├── libdora_node_api_cxx.a
    └── libdora_operator_api_cxx.a
```

> **Note:** The C++ API uses [cxx](https://cxx.rs/) for Rust-C++ interop. The `dora-node-api-cxx_CXX_BRIDGE_FILES` CMake variable (set by `find_package`) points to the cxxbridge source that must be compiled alongside your code.

## Using `dora new`

The easiest way to get started is with the Dora CLI. It generates project scaffolding with `CMakeLists.txt` ready for `find_package`.

### Single Node

```bash
# Create a C node
dora new my-robot --kind node --lang c
cd my-robot
mkdir build && cd build
cmake .. -DCMAKE_PREFIX_PATH=/path/to/dora-c-libraries-<target>
cmake --build .

# Create a C++ node
dora new my-robot --kind node --lang cxx
cd my-robot
mkdir build && cd build
cmake .. -DCMAKE_PREFIX_PATH=/path/to/dora-cpp-libraries-<target>
cmake --build .
```

### Dataflow

```bash
# Create a C dataflow (3 nodes: talker_1, talker_2, listener_1)
dora new my-dataflow --kind dataflow --lang c
cd my-dataflow
mkdir build && cd build
cmake .. -DCMAKE_PREFIX_PATH=/path/to/dora-c-libraries-<target>
cmake --build .

# Create a C++ dataflow
dora new my-dataflow --kind dataflow --lang cxx
cd my-dataflow
mkdir build && cd build
cmake .. -DCMAKE_PREFIX_PATH=/path/to/dora-cpp-libraries-<target>
cmake --build .
```

## CMake Integration

### C Node

```cmake
cmake_minimum_required(VERSION 3.15)
project(my_dora_node C)

find_package(dora-node-api-c REQUIRED)

add_executable(my_node main.c)
target_link_libraries(my_node PRIVATE
    dora-node-api-c::dora-node-api-c
    dora-node-api-c::extra)
```

### C++ Node

```cmake
cmake_minimum_required(VERSION 3.15)
project(my_dora_node CXX)
set(CMAKE_CXX_STANDARD 20)

find_package(dora-node-api-cxx REQUIRED)

add_executable(my_node main.cpp ${dora-node-api-cxx_CXX_BRIDGE_FILES})
target_link_libraries(my_node PRIVATE
    dora-node-api-cxx::dora-node-api-cxx
    dora-node-api-cxx::extra)
```

### Build

```bash
# For C projects
cmake .. -DCMAKE_PREFIX_PATH=/path/to/dora-c-libraries-<target>

# For C++ projects
cmake .. -DCMAKE_PREFIX_PATH=/path/to/dora-cpp-libraries-<target>

cmake --build . --config Release
```

## Manual Linking (Advanced)

If you do not use CMake `find_package`, you can link manually. The targets (`dora-node-api-c::extra` etc.) handle platform-specific system libraries automatically.

### GCC/Clang (Linux)

```bash
# C node
gcc -o my_node main.c \
    -Idora-c-libraries-x86_64-unknown-linux-gnu/include \
    -Ldora-c-libraries-x86_64-unknown-linux-gnu/lib \
    -ldora_node_api_c -lpthread -ldl -lm -lrt

# C++ node (must compile the cxxbridge source too)
g++ -std=c++20 -o my_node main.cpp \
    -Idora-cpp-libraries-x86_64-unknown-linux-gnu/include \
    -Ldora-cpp-libraries-x86_64-unknown-linux-gnu/lib \
    -ldora_node_api_cxx -lpthread -ldl -lm -lrt
```

### Clang (macOS)

```bash
clang++ -std=c++20 -o my_node main.cpp \
    -Idora-cpp-libraries-aarch64-apple-darwin/include \
    -Ldora-cpp-libraries-aarch64-apple-darwin/lib \
    -ldora_node_api_cxx \
    -framework CoreServices -framework Security -lpthread -lm -lc -lresolv
```

## Building from Source

See [Method 2: Build with `xtask`](#method-2-build-with-xtask) above for the complete build and staging workflow.

### Build Output Structure

After `cargo build --release`, build artifacts are in `target/release/` (C crates) or `target/cxxbridge/<crate>/` (C++ crates):

```
target/release/
├── libdora_node_api_c.a
├── dora-node-api-c/
│   ├── lib/cmake/dora-node-api-c/*.cmake
│   └── include/node_api.h
├── libdora_operator_api_c.a
└── dora-operator-api-c/
    ├── lib/cmake/dora-operator-api-c/*.cmake
    └── include/{operator_api.h, operator_types.h}
```

> **Note:** The `xtask stage` command copies artifacts from these locations to a unified prefix directory matching the download archive structure.

## Versioning

Always use libraries that match the version of the Dora runtime you're using.

## Examples

See the repository for complete examples:
- [`examples/c++-dataflow/`](https://github.com/dora-rs/dora/tree/main/examples/c++-dataflow) - C++ API
- [`examples/cmake-dataflow/`](https://github.com/dora-rs/dora/tree/main/examples/cmake-dataflow) - CMake integration
