# Slim version of examples/cmake-dataflow/DoraTargets.cmake — only the
# dora-node-api-c parts (the lifecycle test fixture doesn't need the
# cxx bridge or the operator API). Drives `cargo build -p
# dora-node-api-c` against the dora workspace root specified via
# -DDORA_ROOT_DIR=<...>.

set(DORA_ROOT_DIR "" CACHE FILEPATH "Path to the root of dora")

set(dora_c_include_dir "${CMAKE_CURRENT_BINARY_DIR}/include/c")

if(NOT DORA_ROOT_DIR)
    message(FATAL_ERROR "DORA_ROOT_DIR must be set; pass -DDORA_ROOT_DIR=<path-to-dora-root> at configure time")
endif()

include(ExternalProject)
ExternalProject_Add(Dora
    SOURCE_DIR ${DORA_ROOT_DIR}
    BUILD_IN_SOURCE True
    CONFIGURE_COMMAND ""
    BUILD_COMMAND
        cargo build
        --package dora-node-api-c
    INSTALL_COMMAND ""
)

add_custom_command(OUTPUT ${dora_c_include_dir}
    WORKING_DIRECTORY ${DORA_ROOT_DIR}
    DEPENDS Dora
    COMMAND
        mkdir -p ${CMAKE_CURRENT_BINARY_DIR}/include/c
        &&
        cp -r apis/c/node ${CMAKE_CURRENT_BINARY_DIR}/include/c
)

add_custom_target(Dora_c DEPENDS ${dora_c_include_dir})
set(dora_link_dirs ${DORA_ROOT_DIR}/target/debug)
