cmake_minimum_required(VERSION 2.8.4)
project("UDP Test" C)

add_executable(udp-perf-client perf-client.cpp)

add_executable(udp-perf-server perf-server.cpp)


add_executable(udp-multi-server multi-server.cpp)
target_link_libraries(udp-multi-server pthread)

add_executable(udp-multi-client multi-client.cpp)
target_link_libraries(udp-multi-client pthread)