cmake_minimum_required(VERSION 3.15)
project(cmake-arrow-dataflow LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS "-fPIC")

find_package(Arrow REQUIRED)

include(DoraTargets.cmake)

link_directories(${dora_link_dirs})

add_executable(node node/main.cc ${node_bridge})
add_dependencies(node Dora_cxx)
target_include_directories(node PRIVATE ${dora_cxx_include_dir})
target_link_libraries(node dora_node_api_cxx Arrow::arrow_shared)

# Platform-specific system libraries required by the dora static library
if(APPLE)
    target_link_libraries(node
        "-framework CoreServices" "-framework CoreFoundation" "-framework Security"
        pthread m c resolv)
elseif(UNIX)
    target_link_libraries(node pthread dl m rt)
elseif(WIN32)
    target_link_libraries(node
        advapi32 userenv kernel32 ws2_32 bcrypt ncrypt schannel ntdll iphlpapi
        cfgmgr32 credui crypt32 cryptnet fwpuclnt gdi32 msimg32 mswsock ole32
        oleaut32 opengl32 secur32 shell32 synchronization user32 winspool)
endif()

install(TARGETS node DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/build)
