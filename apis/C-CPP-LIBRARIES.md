# Using Pre-built Dora C/C++ Libraries

Pre-built C and C++ static libraries are published as GitHub release artifacts for each Dora release.

## Available Libraries

### C API
- **dora-node-api-c** - C API for creating Dora nodes
- **dora-operator-api-c** - C API for creating Dora operators

### C++ API
- **dora-node-api-cxx** - C++ API for creating Dora nodes (cxx bridge)
- **dora-operator-api-cxx** - C++ API for creating Dora operators (cxx bridge)

## Supported Platforms

| Platform | Architecture | Archive |
|---|---|---|
| Linux | x86_64 | `.tar.gz` |
| Linux | aarch64 (ARM64) | `.tar.gz` |
| macOS | aarch64 (Apple Silicon) | `.tar.gz` |
| Windows | x86_64 | `.zip` |

## Download

1. Go to the [Dora releases page](https://github.com/dora-rs/dora/releases)
2. Download the archive for your platform:
   - C API: `dora-c-libraries-<platform>-<arch>.tar.gz` (or `.zip` on Windows)
   - C++ API: `dora-cpp-libraries-<platform>-<arch>.tar.gz` (or `.zip` on Windows)

```bash
# Example: download and extract C++ libraries for Linux x86_64
tar xzf dora-cpp-libraries-linux-x86_64.tar.gz
```

## Directory Structure

**C libraries:**
```
dora-c-libraries-<platform>-<arch>/
├── include/
│   ├── node_api.h
│   ├── operator_api.h
│   └── operator_types.h
└── lib/
    ├── libdora_node_api_c.a          # Unix
    └── libdora_operator_api_c.a      # Unix
    # or dora_node_api_c.lib etc. on Windows
```

**C++ libraries:**
```
dora-cpp-libraries-<platform>-<arch>/
├── include/
│   ├── dora-node-api.h
│   └── dora-operator-api.h
├── src/
│   ├── dora-node-api.cc              # Bridge source (compile with your code)
│   └── dora-operator-api.cc
└── lib/
    ├── libdora_node_api_cxx.a        # Unix
    └── libdora_operator_api_cxx.a    # Unix
```

> **Note:** The C++ API uses [cxx](https://cxx.rs/) for Rust-C++ interop. The `src/*.cc` bridge files must be compiled alongside your code -- they are not included in the static library.

## CMake Integration

### C Node Example

```cmake
cmake_minimum_required(VERSION 3.15)
project(my_dora_node C)

# Detect platform for the correct library directory
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
        set(DORA_PLATFORM "linux-aarch64")
    else()
        set(DORA_PLATFORM "linux-x86_64")
    endif()
elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
    set(DORA_PLATFORM "macos-aarch64")
elseif(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    set(DORA_PLATFORM "windows-x86_64")
endif()

set(DORA_C_DIR "${CMAKE_SOURCE_DIR}/dora-c-libraries-${DORA_PLATFORM}")

add_executable(my_node main.c)
target_include_directories(my_node PRIVATE ${DORA_C_DIR}/include)

if(WIN32)
    target_link_libraries(my_node ${DORA_C_DIR}/lib/dora_node_api_c.lib
        advapi32 userenv kernel32 ws2_32 bcrypt ncrypt schannel ntdll iphlpapi
        cfgmgr32 credui crypt32 cryptnet fwpuclnt gdi32 msimg32 mswsock ole32
        oleaut32 opengl32 secur32 shell32 synchronization user32 winspool)
elseif(APPLE)
    target_link_libraries(my_node ${DORA_C_DIR}/lib/libdora_node_api_c.a
        "-framework CoreServices" "-framework Security"
        pthread m c resolv)
else()
    target_link_libraries(my_node ${DORA_C_DIR}/lib/libdora_node_api_c.a
        pthread dl m rt)
endif()
```

### C++ Node Example

```cmake
cmake_minimum_required(VERSION 3.15)
project(my_dora_node CXX)
set(CMAKE_CXX_STANDARD 20)

# ... same platform detection as above ...

set(DORA_CPP_DIR "${CMAKE_SOURCE_DIR}/dora-cpp-libraries-${DORA_PLATFORM}")

# The bridge source file must be compiled with your code
add_executable(my_node
    main.cpp
    ${DORA_CPP_DIR}/src/dora-node-api.cc
)
target_include_directories(my_node PRIVATE ${DORA_CPP_DIR}/include)

if(WIN32)
    target_link_libraries(my_node ${DORA_CPP_DIR}/lib/dora_node_api_cxx.lib
        advapi32 userenv kernel32 ws2_32 bcrypt ncrypt schannel ntdll iphlpapi
        cfgmgr32 credui crypt32 cryptnet fwpuclnt gdi32 msimg32 mswsock ole32
        oleaut32 opengl32 secur32 shell32 synchronization user32 winspool)
elseif(APPLE)
    target_link_libraries(my_node ${DORA_CPP_DIR}/lib/libdora_node_api_cxx.a
        "-framework CoreServices" "-framework Security"
        pthread m c resolv)
else()
    target_link_libraries(my_node ${DORA_CPP_DIR}/lib/libdora_node_api_cxx.a
        pthread dl m rt)
endif()
```

## Manual Linking

### GCC/Clang (Linux)

```bash
# C node
gcc -o my_node main.c \
    -Idora-c-libraries-linux-x86_64/include \
    -Ldora-c-libraries-linux-x86_64/lib \
    -ldora_node_api_c -lpthread -ldl -lm -lrt

# C++ node (must compile the bridge source too)
g++ -std=c++20 -o my_node main.cpp dora-cpp-libraries-linux-x86_64/src/dora-node-api.cc \
    -Idora-cpp-libraries-linux-x86_64/include \
    -Ldora-cpp-libraries-linux-x86_64/lib \
    -ldora_node_api_cxx -lpthread -ldl -lm -lrt
```

### Clang (macOS)

```bash
# C++ node
clang++ -std=c++20 -o my_node main.cpp dora-cpp-libraries-macos-aarch64/src/dora-node-api.cc \
    -Idora-cpp-libraries-macos-aarch64/include \
    -Ldora-cpp-libraries-macos-aarch64/lib \
    -ldora_node_api_cxx \
    -framework CoreServices -framework Security -lpthread -lm -lc -lresolv
```

## Building from Source

If you prefer to build the libraries yourself:

```bash
git clone https://github.com/dora-rs/dora.git
cd dora

cargo build --release \
    -p dora-node-api-c \
    -p dora-operator-api-c \
    -p dora-node-api-cxx \
    -p dora-operator-api-cxx

# Static libraries:   target/release/
# C headers:          apis/c/node/, apis/c/operator/
# C++ bridge files:   target/cxxbridge/dora-{node,operator}-api-cxx/src/lib.rs.{h,cc}
```

## Versioning

Always use libraries that match the version of the Dora runtime you're using.

## Examples

See the repository for complete examples:
- [`examples/c++-dataflow/`](https://github.com/dora-rs/dora/tree/main/examples/c++-dataflow) - C++ API
- [`examples/cmake-dataflow/`](https://github.com/dora-rs/dora/tree/main/examples/cmake-dataflow) - CMake integration
