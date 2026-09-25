cmake_minimum_required(VERSION 3.14)
project(SerialEventLoop LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

get_filename_component(DORA_DEFAULT_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
set(DORA_ROOT "${DORA_DEFAULT_ROOT}" CACHE PATH "Path to the Dora repository root")
set(DORA_INCLUDE_DIR "${DORA_ROOT}/apis/c/node")
set(DORA_LIB_PATH
    "${DORA_ROOT}/target/release/${CMAKE_STATIC_LIBRARY_PREFIX}dora_node_api_c${CMAKE_STATIC_LIBRARY_SUFFIX}"
    CACHE FILEPATH "Path to the Dora C node API static library")

add_executable(serial_event_loop_test test_serial_event_loop.cpp)
target_include_directories(serial_event_loop_test PRIVATE ${DORA_INCLUDE_DIR})
target_link_libraries(serial_event_loop_test
    m
    dl
    pthread
)

enable_testing()
add_test(NAME serial_event_loop_test COMMAND serial_event_loop_test)

add_executable(cpp_sender sender.cpp)
add_executable(cpp_receiver receiver.cpp)

foreach(target cpp_sender cpp_receiver)
    target_include_directories(${target} PRIVATE ${DORA_INCLUDE_DIR})
    target_link_libraries(${target} PRIVATE
        ${DORA_LIB_PATH}
        m
        dl
        pthread
    )
endforeach()
