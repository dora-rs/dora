cmake_minimum_required(VERSION 3.15)
project(dora-find-package-smoke-c LANGUAGES C)

# Exercise the find_package contract emitted by the dora-node-api-c
# build.rs + xtask stage pipeline. Failing to find the package is the
# whole point — it means the cmake config files weren't generated or
# weren't staged where consumers expect.
find_package(dora-node-api-c REQUIRED)

add_executable(smoke main.c)
target_link_libraries(smoke
    PRIVATE
        dora-node-api-c::dora-node-api-c
        dora-node-api-c::extra
)
