cmake_minimum_required(VERSION 3.15)
project(dora-find-package-smoke-operator-c LANGUAGES C)

# Exercise the find_package contract for the dora-operator-api-c
# crate. Operators are loaded as shared libraries by the dora runtime,
# so the *consumer* defines `dora_init_operator` / `dora_drop_operator`
# / `dora_on_event` and the library provides the type definitions +
# linkage. A shared-library target keeps the smoke close to how a real
# operator gets built.
find_package(dora-operator-api-c REQUIRED)

add_library(smoke SHARED main.c)
target_link_libraries(smoke
    PRIVATE
        dora-operator-api-c::dora-operator-api-c
        dora-operator-api-c::extra
)
