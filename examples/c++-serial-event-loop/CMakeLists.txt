cmake_minimum_required(VERSION 3.14)
project(SerialEventLoop LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

 
set(DORA_ROOT /home/mickrobot/dora)
set(DORA_INCLUDE_DIR ${DORA_ROOT}/apis/c/node; ${DORA_ROOT}/apis/c/operator)
set(DORA_LIB_PATH ${DORA_ROOT}/target/release/libdora_node_api_c.a)

include_directories( 
    ${DORA_INCLUDE_DIR}
)

add_executable(serial_event_loop_test test_serial_event_loop.cpp)
target_link_libraries(serial_event_loop_test
    ${DORA_LIB_PATH} 
    m
    dl
    pthread
)

add_executable(cpp_sender sender.cpp)
add_executable(cpp_receiver receiver.cpp)

foreach(target cpp_sender cpp_receiver)
    target_link_libraries(${target} PRIVATE
        ${DORA_LIB_PATH}
        m
        dl
        pthread
    )
endforeach()
