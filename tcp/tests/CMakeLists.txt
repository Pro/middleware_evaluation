cmake_minimum_required(VERSION 2.8.4)
project("TCP Test" C)

add_executable(tcp-perf-client perf-client.cpp)

add_executable(tcp-perf-server perf-server.cpp)


add_executable(tcp-multi-server multi-server.cpp)
target_link_libraries(tcp-multi-server pthread)

add_executable(tcp-multi-client multi-client.cpp)
target_link_libraries(tcp-multi-client pthread)