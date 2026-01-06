# Using Dora C/C++ Libraries

This document explains how to use the pre-built C and C++ libraries published with each Dora release.

## Available Libraries

Dora provides the following C/C++ libraries:

### C API
- **dora-node-api-c**: C API for creating Dora nodes
- **dora-operator-api-c**: C API for creating Dora operators

### C++ API
- **dora-node-api-cxx**: C++ API for creating Dora nodes (using cxx bridge)
- **dora-operator-api-cxx**: C++ API for creating Dora operators (using cxx bridge)

## Download Pre-built Libraries

Pre-built libraries are published as GitHub release artifacts for each Dora release. The libraries are distributed in two separate archives:

- **dora-c-libraries.tar.gz**: C API libraries for all platforms
- **dora-cpp-libraries.tar.gz**: C++ API libraries for all platforms

### Supported Platforms

Each archive contains libraries for the following platforms:

- **Linux**: `x86_64`, `aarch64` (ARM64)
- **macOS**: `aarch64` (Apple Silicon)
- **Windows**: `x86_64`

### Downloading

1. Go to the [Dora releases page](https://github.com/dora-rs/dora/releases)
2. Select the release version you want to use
3. Download the appropriate archive(s):
   - For C API: `dora-c-libraries.tar.gz`
   - For C++ API: `dora-cpp-libraries.tar.gz`
   - Or download both if you need both APIs

### Extracting

```bash
# Extract C libraries
tar xzf dora-c-libraries.tar.gz
cd c-libs-<platform>-<arch>/

# Or extract C++ libraries
tar xzf dora-cpp-libraries.tar.gz
cd cpp-libs-<platform>-<arch>/
```

Replace `<platform>-<arch>` with your platform:
- `linux-x86_64`
- `linux-aarch64`
- `macos-aarch64`
- `windows-x86_64`

## Using the Libraries

### Directory Structure

**C Libraries Archive** (`dora-c-libraries.tar.gz`):
```
c-libs-<platform>-<arch>/
├── libdora_node_api_c.a          # C node API static library (Unix)
├── dora_node_api_c.lib           # C node API static library (Windows)
├── node_api.h                    # C node API header
├── libdora_operator_api_c.a      # C operator API static library (Unix)
├── dora_operator_api_c.lib       # C operator API static library (Windows)
├── operator_api.h                # C operator API header
└── operator_types.h              # C operator types header
```

**C++ Libraries Archive** (`dora-cpp-libraries.tar.gz`):
```
cpp-libs-<platform>-<arch>/
├── libdora_node_api_cxx.a        # C++ node API static library (Unix)
├── dora_node_api_cxx.lib         # C++ node API static library (Windows)
├── libdora_operator_api_cxx.a    # C++ operator API static library (Unix)
├── dora_operator_api_cxx.lib     # C++ operator API static library (Windows)
└── *.h                           # C++ generated headers from cxx bridge
```

### Linking with CMake

Example `CMakeLists.txt` for a C node:

```cmake
cmake_minimum_required(VERSION 3.15)
project(my_dora_node)

# Detect platform
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
        set(PLATFORM "linux-aarch64")
    else()
        set(PLATFORM "linux-x86_64")
    endif()
elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
    set(PLATFORM "macos-aarch64")
elseif(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    set(PLATFORM "windows-x86_64")
endif()

# Set the path to the extracted C libraries
set(DORA_C_LIB_PATH "${CMAKE_SOURCE_DIR}/c-libs-${PLATFORM}")

# Add include directories
include_directories(${DORA_C_LIB_PATH})

# Create your executable
add_executable(my_node main.c)

# Link against Dora C API
if(UNIX)
    target_link_libraries(my_node ${DORA_C_LIB_PATH}/libdora_node_api_c.a)
else()
    target_link_libraries(my_node ${DORA_C_LIB_PATH}/dora_node_api_c.lib)
endif()

# You may need to link additional system libraries
if(UNIX AND NOT APPLE)
    target_link_libraries(my_node pthread dl m)
endif()

if(APPLE)
    target_link_libraries(my_node "-framework CoreServices" "-framework Security")
endif()
```

For C++ nodes:

```cmake
# Set the path to the extracted C++ libraries
set(DORA_CPP_LIB_PATH "${CMAKE_SOURCE_DIR}/cpp-libs-${PLATFORM}")

# Add include directories
include_directories(${DORA_CPP_LIB_PATH})

# C++ node example
add_executable(my_cpp_node main.cpp)

if(UNIX)
    target_link_libraries(my_cpp_node ${DORA_CPP_LIB_PATH}/libdora_node_api_cxx.a)
else()
    target_link_libraries(my_cpp_node ${DORA_CPP_LIB_PATH}/dora_node_api_cxx.lib)
endif()

# Standard C++ and system libraries
target_link_libraries(my_cpp_node stdc++)
if(UNIX AND NOT APPLE)
    target_link_libraries(my_cpp_node pthread dl m)
endif()
```

### Linking Manually

**GCC/Clang (Linux/macOS):**
```bash
# C node
gcc -o my_node main.c -Ic-libs-linux-x86_64 -Lc-libs-linux-x86_64 -ldora_node_api_c -lpthread -ldl -lm

# C++ node
g++ -o my_node main.cpp -Icpp-libs-linux-x86_64 -Lcpp-libs-linux-x86_64 -ldora_node_api_cxx -lpthread -ldl -lm
```

**MSVC (Windows):**
```cmd
REM C node
cl /I c-libs-windows-x86_64 main.c /link c-libs-windows-x86_64\dora_node_api_c.lib

REM C++ node
cl /I cpp-libs-windows-x86_64 main.cpp /link cpp-libs-windows-x86_64\dora_node_api_cxx.lib
```

## Examples

See the `examples/` directory in the Dora repository for complete examples:
- `examples/c-dataflow/` - C API examples
- `examples/c++-dataflow/` - C++ API examples
- `examples/cmake-dataflow/` - CMake build examples

## Building from Source

If you prefer to build the libraries yourself:

```bash
# Clone the repository
git clone https://github.com/dora-rs/dora.git
cd dora

# Build C libraries
cargo build --release --package dora-node-api-c
cargo build --release --package dora-operator-api-c

# Build C++ libraries
cargo build --release --package dora-node-api-cxx
cargo build --release --package dora-operator-api-cxx

# Libraries will be in target/release/
# Headers are in apis/c/node/, apis/c/operator/, and target/cxxbridge/
```

## Versioning

The C/C++ libraries follow the same version as the Dora release. Always use libraries that match the version of the Dora runtime you're using.

## Support

For issues or questions:
- GitHub Issues: https://github.com/dora-rs/dora/issues
- Documentation: https://dora-rs.ai/docs

## License

The Dora C/C++ libraries are licensed under Apache-2.0, same as the Dora project.
