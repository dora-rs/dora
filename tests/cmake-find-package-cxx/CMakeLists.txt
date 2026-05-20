cmake_minimum_required(VERSION 3.15)
project(dora-find-package-smoke-cxx LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)

# Exercise the find_package contract for the dora-node-api-cxx
# (cxxbridge) crate. Unlike the C API, consumers must compile the
# generated cxxbridge .cc alongside their own sources — the package's
# CMake config exports that path via `${dora-node-api-cxx_CXX_BRIDGE_FILES}`.
find_package(dora-node-api-cxx REQUIRED)

add_executable(smoke
    main.cc
    ${dora-node-api-cxx_CXX_BRIDGE_FILES}
)
target_link_libraries(smoke
    PRIVATE
        dora-node-api-cxx::dora-node-api-cxx
        dora-node-api-cxx::extra
)
