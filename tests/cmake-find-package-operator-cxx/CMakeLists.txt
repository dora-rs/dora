cmake_minimum_required(VERSION 3.15)
project(dora-find-package-smoke-operator-cxx LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)

# Exercise the find_package contract for the dora-operator-api-cxx
# (cxxbridge) crate. The cxxbridge generated .cc is exported via
# `${dora-operator-api-cxx_CXX_BRIDGE_FILES}` and includes operator.h
# from this directory, so the source fileset must build alongside it.
find_package(dora-operator-api-cxx REQUIRED)

add_library(smoke SHARED
    main.cc
    ${dora-operator-api-cxx_CXX_BRIDGE_FILES}
)

# The generated cxxbridge .cc does `#include "operator.h"` — make
# our minimal stub discoverable from the bridge translation unit.
target_include_directories(smoke PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(smoke
    PRIVATE
        dora-operator-api-cxx::dora-operator-api-cxx
        dora-operator-api-cxx::extra
)
