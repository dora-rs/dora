cmake_minimum_required(VERSION 3.21)
project(cxx-dynamic-add-remove LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-fPIC")

# DoraTargets.cmake drives `cargo build -p dora-node-api-c` via
# ExternalProject_Add and exposes:
#   - dora_c_include_dir : path to the copied C header
#   - dora_link_dirs      : path to target/debug (where libdora_node_api_c
#                          lives)
include(DoraTargets.cmake)

link_directories(${dora_link_dirs})

# System libs needed to link `libdora_node_api_c.a` — the static
# archive transitively pulls in iana-time-zone (CoreFoundation on
# macOS), zenoh (BSD sockets + pthread + dl), etc. The list mirrors
# what `apis/c/node/cmake/dora-api-config.cmake.in` exports via the
# `@PACKAGE@::extra` interface library.
if(APPLE)
    set(dora_system_libs
        "-framework CoreServices" "-framework Security"
        "-framework CoreFoundation"
        pthread m c resolv)
elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set(dora_system_libs pthread dl m rt)
else()
    set(dora_system_libs "")
endif()

# Three minimal C-API nodes used by tests/node-lifecycle-e2e.rs
# (issue #1703, language variant 3).

add_executable(cxx_sender sender.cc)
add_dependencies(cxx_sender Dora_c)
target_include_directories(cxx_sender PRIVATE ${dora_c_include_dir})
target_link_libraries(cxx_sender dora_node_api_c ${dora_system_libs})

add_executable(cxx_receiver receiver.cc)
add_dependencies(cxx_receiver Dora_c)
target_include_directories(cxx_receiver PRIVATE ${dora_c_include_dir})
target_link_libraries(cxx_receiver dora_node_api_c ${dora_system_libs})

add_executable(cxx_filter filter.cc)
add_dependencies(cxx_filter Dora_c)
target_include_directories(cxx_filter PRIVATE ${dora_c_include_dir})
target_link_libraries(cxx_filter dora_node_api_c ${dora_system_libs})
