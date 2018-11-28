cmake_minimum_required(VERSION 2.8.4)

#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fno-omit-frame-pointer -gline-tables-only -fsanitize=address -fsanitize-address-use-after-scope -fsanitize-coverage=trace-pc-guard,trace-cmp")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-omit-frame-pointer -gline-tables-only -fsanitize=address -fsanitize-address-use-after-scope -fsanitize-coverage=trace-pc-guard,trace-cmp")


set(UA_ENABLE_AMALGAMATION OFF)
set(UA_ENABLE_PUBSUB ON CACHE BOOL "" FORCE)
set(UA_ENABLE_PUBSUB_INFORMATIONMODEL ON CACHE BOOL "" FORCE)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_subdirectory(libs/open62541)
include_directories(${CMAKE_BINARY_DIR}/libs/open62541)

add_subdirectory(libs/paho.mqtt.c)
include_directories(libs/paho.mqtt.c/src)

add_subdirectory(libs/fastrtps)


include_directories(common)

add_subdirectory(mqtt/tests)
add_subdirectory(opc-ua/tests)
add_subdirectory(dds/tests)
#add_subdirectory(opendds/tests)
add_subdirectory(tcp/tests)
add_subdirectory(udp/tests)