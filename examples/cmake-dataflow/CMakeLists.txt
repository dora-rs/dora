cmake_minimum_required(VERSION 3.21)
project(cmake-dataflow LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-fPIC")

include(DoraTargets.cmake)

link_directories(${dora_link_dirs})

add_executable(node_c_api node-c-api/main.cc)
add_dependencies(node_c_api Dora_c)
target_include_directories(node_c_api PRIVATE ${dora_c_include_dir})
target_link_libraries(node_c_api dora_node_api_c
m
rt
dl
pthread
pcap	
)

add_executable(node_rust_api node-rust-api/main.cc ${node_bridge})
add_dependencies(node_rust_api Dora_cxx)
target_include_directories(node_rust_api PRIVATE ${dora_cxx_include_dir})
target_link_libraries(node_rust_api dora_node_api_cxx
m
rt
dl
pthread
pcap
)

add_library(operator_c_api SHARED operator-c-api/operator.cc)
add_dependencies(operator_c_api Dora_c)
target_include_directories(operator_c_api PRIVATE ${dora_c_include_dir})
target_link_libraries(operator_c_api dora_operator_api_c)

add_library(operator_rust_api SHARED operator-rust-api/operator.cc ${operator_bridge})
add_dependencies(operator_rust_api Dora_cxx)
target_include_directories(operator_rust_api PRIVATE ${dora_cxx_include_dir} ${dora_c_include_dir} ${CMAKE_CURRENT_SOURCE_DIR}/operator-rust-api)
target_link_libraries(operator_rust_api dora_operator_api_cxx)

install(TARGETS node_c_api node_rust_api DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/bin)
install(TARGETS operator_c_api operator_rust_api DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/lib)
