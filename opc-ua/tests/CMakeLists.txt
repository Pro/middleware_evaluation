cmake_minimum_required(VERSION 2.8.4)
project("OPC UA Test" C)

include_directories(${CMAKE_BINARY_DIR}/libs/open62541)
include_directories(${CMAKE_BINARY_DIR}/libs/open62541/src_generated)


include_directories(${CMAKE_SOURCE_DIR}/libs/open62541/deps)
include_directories(${CMAKE_SOURCE_DIR}/libs/open62541/arch)
include_directories(${CMAKE_SOURCE_DIR}/libs/open62541/arch/posix)
include_directories(${CMAKE_SOURCE_DIR}/libs/open62541/include)
include_directories(${CMAKE_SOURCE_DIR}/libs/open62541/plugins)
include_directories(${CMAKE_SOURCE_DIR}/libs/open62541/src)
include_directories(${CMAKE_SOURCE_DIR}/libs/open62541/src/pubsub)
include_directories(${CMAKE_SOURCE_DIR}/libs/open62541/src/server)


add_executable(opcua-perf-client perf-client.c)
target_link_libraries(opcua-perf-client open62541)

add_executable(opcua-perf-server perf-server.c)
target_link_libraries(opcua-perf-server open62541)

add_executable(opcua-multi-server multiple-server.c)
target_link_libraries(opcua-multi-server open62541 pthread)

add_executable(opcua-multi-client multiple-client.c)
target_link_libraries(opcua-multi-client open62541 pthread)

add_executable(opcua-overhead-client overhead-client.c)
target_link_libraries(opcua-overhead-client open62541)

add_executable(opcua-overhead-server overhead-server.c)
target_link_libraries(opcua-overhead-server open62541)

add_executable(opcua-perf-ps-client perf-ps-client.c)
target_link_libraries(opcua-perf-ps-client open62541)

add_executable(opcua-multi-ps-client multi-ps-client.c)
target_link_libraries(opcua-multi-ps-client open62541)

add_executable(opcua-perf-ps-server perf-ps-server.c)
target_link_libraries(opcua-perf-ps-server open62541)
