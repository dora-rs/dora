set(DORA_ROOT_DIR "" CACHE FILEPATH "Path to the root of dora")

set(dora_c_include_dir "${CMAKE_CURRENT_BINARY_DIR}/include/c")

set(dora_cxx_include_dir "${CMAKE_CURRENT_BINARY_DIR}/include/cxx")
set(node_bridge "${CMAKE_CURRENT_BINARY_DIR}/node_bridge.cc")
set(operator_bridge "${CMAKE_CURRENT_BINARY_DIR}/operator_bridge.cc")

if(DORA_ROOT_DIR)
    include(ExternalProject)
    ExternalProject_Add(Dora
        SOURCE_DIR ${DORA_ROOT_DIR}
        BUILD_IN_SOURCE True
        CONFIGURE_COMMAND ""
        BUILD_COMMAND
            cargo build
            --package dora-node-api-c
            &&
            cargo build
            --package dora-operator-api-c
            &&
            cargo build
            --package dora-node-api-cxx
            &&
            cargo build
            --package dora-operator-api-cxx
        INSTALL_COMMAND ""
    )
    
    add_custom_command(OUTPUT ${node_bridge} ${dora_cxx_include_dir} ${operator_bridge} ${dora_c_include_dir}
        WORKING_DIRECTORY ${DORA_ROOT_DIR}
        DEPENDS Dora
        COMMAND
            mkdir ${dora_cxx_include_dir} -p
            &&
            mkdir ${CMAKE_CURRENT_BINARY_DIR}/include/c -p
            &&
            cp target/cxxbridge/dora-node-api-cxx/src/lib.rs.cc ${node_bridge}
            &&
            cp target/cxxbridge/dora-node-api-cxx/src/lib.rs.h ${dora_cxx_include_dir}/dora-node-api.h
            &&
            cp target/cxxbridge/dora-operator-api-cxx/src/lib.rs.cc ${operator_bridge}
            &&
            cp target/cxxbridge/dora-operator-api-cxx/src/lib.rs.h ${dora_cxx_include_dir}/dora-operator-api.h
            &&
            cp apis/c/node ${CMAKE_CURRENT_BINARY_DIR}/include/c -r
            &&
            cp apis/c/operator ${CMAKE_CURRENT_BINARY_DIR}/include/c -r

    )
    
    add_custom_target(Dora_c DEPENDS ${dora_c_include_dir})
    add_custom_target(Dora_cxx DEPENDS ${node_bridge} ${operator_bridge} ${dora_cxx_include_dir})
    set(dora_link_dirs ${DORA_ROOT_DIR}/target/debug)
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
            --target-dir ${CMAKE_CURRENT_BINARY_DIR}/dora/src/Dora/target
            &&
            cargo build
            --package dora-operator-api-c
            --target-dir ${CMAKE_CURRENT_BINARY_DIR}/dora/src/Dora/target
            &&
            cargo build
            --package dora-node-api-cxx
            --target-dir ${CMAKE_CURRENT_BINARY_DIR}/dora/src/Dora/target
            &&
            cargo build
            --package dora-operator-api-cxx
            --target-dir ${CMAKE_CURRENT_BINARY_DIR}/dora/src/Dora/target
        INSTALL_COMMAND ""
    )

    add_custom_command(OUTPUT ${node_bridge} ${dora_cxx_include_dir} ${operator_bridge} ${dora_c_include_dir}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/dora/src/Dora/target
        DEPENDS Dora
        COMMAND
            mkdir ${CMAKE_CURRENT_BINARY_DIR}/include/c -p
            &&
            mkdir ${dora_cxx_include_dir} -p
            &&
            cp cxxbridge/dora-node-api-cxx/src/lib.rs.cc ${node_bridge}
            &&
            cp cxxbridge/dora-node-api-cxx/src/lib.rs.h ${dora_cxx_include_dir}/dora-node-api.h
            &&
            cp cxxbridge/dora-operator-api-cxx/src/lib.rs.cc ${operator_bridge}
            &&
            cp cxxbridge/dora-operator-api-cxx/src/lib.rs.h ${dora_cxx_include_dir}/dora-operator-api.h
            &&
            cp ../apis/c/node ${CMAKE_CURRENT_BINARY_DIR}/include/c -r
            &&
            cp ../apis/c/operator ${CMAKE_CURRENT_BINARY_DIR}/include/c -r
    )

    set(dora_link_dirs ${CMAKE_CURRENT_BINARY_DIR}/dora/src/Dora/target/debug)
    
    add_custom_target(Dora_c DEPENDS ${dora_c_include_dir})
    add_custom_target(Dora_cxx DEPENDS ${node_bridge} ${operator_bridge} ${dora_cxx_include_dir})
endif()


