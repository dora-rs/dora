set(DORA_ROOT_DIR "" CACHE FILEPATH "Path to the root of dora")

set(dora_c_include_dir "${CMAKE_CURRENT_BINARY_DIR}/include/c")

set(dora_cxx_include_dir "${CMAKE_CURRENT_BINARY_DIR}/include/cxx")
set(node_bridge "${CMAKE_CURRENT_BINARY_DIR}/node_bridge.cc")
set(operator_bridge "${CMAKE_CURRENT_BINARY_DIR}/operator_bridge.cc")

if(DORA_ROOT_DIR)
    include(FetchContent)
    FetchContent_Declare(
        Corrosion
        GIT_REPOSITORY https://github.com/corrosion-rs/corrosion.git
        GIT_TAG v0.4.3
    )
    FetchContent_MakeAvailable(Corrosion)
    list(PREPEND CMAKE_MODULE_PATH ${Corrosion_SOURCE_DIR}/cmake)
    find_package(Rust 1.70 REQUIRED MODULE)
    corrosion_import_crate(MANIFEST_PATH "${DORA_ROOT_DIR}/Cargo.toml"
        CRATES
        dora-node-api-c
        dora-operator-api-c
        CRATE_TYPES
        staticlib staticlib
    )
    add_custom_command(OUTPUT ${dora_c_include_dir}
        WORKING_DIRECTORY ${DORA_ROOT_DIR}/apis/c
        COMMAND
            mkdir ${CMAKE_CURRENT_BINARY_DIR}/include/c -p
            &&
            cp node ${CMAKE_CURRENT_BINARY_DIR}/include/c -r
            &&
            cp operator ${CMAKE_CURRENT_BINARY_DIR}/include/c -r
        DEPENDS dora-node-api-c dora-operator-api-c
    )

    corrosion_import_crate(MANIFEST_PATH "${DORA_ROOT_DIR}/Cargo.toml"
        CRATES
        dora-node-api-cxx
        dora-operator-api-cxx
        CRATE_TYPES
        staticlib staticlib
    )
    add_custom_command(OUTPUT ${node_bridge} ${dora_cxx_include_dir} ${operator_bridge}
        WORKING_DIRECTORY ${DORA_ROOT_DIR}
        DEPENDS dora-node-api-cxx dora-operator-api-cxx
        COMMAND
            mkdir ${dora_cxx_include_dir} -p
            &&
            cp target/cxxbridge/dora-node-api-cxx/src/lib.rs.cc ${node_bridge}
            &&
            cp target/cxxbridge/dora-node-api-cxx/src/lib.rs.h ${dora_cxx_include_dir}/dora-node-api.h
            &&
            cp target/cxxbridge/dora-operator-api-cxx/src/lib.rs.cc ${operator_bridge}
            &&
            cp target/cxxbridge/dora-operator-api-cxx/src/lib.rs.h ${dora_cxx_include_dir}/dora-operator-api.h
    )
    
    add_custom_target(Dora_c DEPENDS dora-node-api-c dora-operator-api-c ${dora_c_include_dir})
    add_custom_target(Dora_cxx DEPENDS dora-node-api-cxx  dora-operator-api-cxx ${node_bridge} ${operator_bridge} ${dora_cxx_include_dir})
    set(dora_link_dirs ${CMAKE_CURRENT_BINARY_DIR})
else()
    include(ExternalProject)
    ExternalProject_Add(Dora
        PREFIX ${CMAKE_CURRENT_BINARY_DIR}/dora
        GIT_REPOSITORY https://github.com/dora-rs/dora.git
        GIT_TAG main
        BUILD_IN_SOURCE True
        CONFIGURE_COMMAND ""
        BUILD_COMMAND
            cargo build
            --package dora-node-api-c
            --target-dir ${CMAKE_CURRENT_BINARY_DIR}
            &&
            cargo build
            --package dora-operator-api-c
            --target-dir ${CMAKE_CURRENT_BINARY_DIR}
            &&
            cargo build
            --package dora-node-api-cxx
            --target-dir ${CMAKE_CURRENT_BINARY_DIR}
            &&
            cargo build
            --package dora-operator-api-cxx
            --target-dir ${CMAKE_CURRENT_BINARY_DIR}
        INSTALL_COMMAND ""
    )
    add_custom_command(OUTPUT ${dora_c_include_dir}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/dora/src/Dora/apis/c
        COMMAND
            mkdir ${CMAKE_CURRENT_BINARY_DIR}/include/c -p
            &&
            cp node ${CMAKE_CURRENT_BINARY_DIR}/include/c -r
            &&
            cp operator ${CMAKE_CURRENT_BINARY_DIR}/include/c -r
        DEPENDS Dora
    )
    add_custom_command(OUTPUT ${node_bridge} ${dora_cxx_include_dir} ${operator_bridge}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        DEPENDS Dora
        COMMAND
            mkdir ${dora_cxx_include_dir} -p
            &&
            cp cxxbridge/dora-node-api-cxx/src/lib.rs.cc ${node_bridge}
            &&
            cp cxxbridge/dora-node-api-cxx/src/lib.rs.h ${dora_cxx_include_dir}/dora-node-api.h
            &&
            cp cxxbridge/dora-operator-api-cxx/src/lib.rs.cc ${operator_bridge}
            &&
            cp cxxbridge/dora-operator-api-cxx/src/lib.rs.h ${dora_cxx_include_dir}/dora-operator-api.h
    )

    set(dora_link_dirs ${CMAKE_CURRENT_BINARY_DIR}/debug)
    
    add_custom_target(Dora_c DEPENDS ${dora_c_include_dir})
    add_custom_target(Dora_cxx DEPENDS ${node_bridge} ${operator_bridge} ${dora_cxx_include_dir})
endif()


