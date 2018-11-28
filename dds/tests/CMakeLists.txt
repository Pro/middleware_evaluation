cmake_minimum_required(VERSION 2.8.4)
project("DDS Test" C)


SET(IDL_SRCS
        idl/DDSPerfTest.cxx
        idl/DDSPerfTestPubSubTypes.cxx
        )
include_directories(idl)

#add_executable(dds-overhead-pub
#        ${IDL_SRCS}
#        Writer.cpp
#        overhead-pub.cpp)
#target_link_libraries(dds-overhead-pub fastrtps fastcdr)
#
#add_executable(dds-overhead-sub
#        ${IDL_SRCS}
#        overhead-sub.cpp)
#
#target_link_libraries(dds-overhead-sub fastrtps fastcdr)
#
add_executable(dds-perf-client
        ${IDL_SRCS}
        perf-client.cpp)
target_link_libraries(dds-perf-client fastrtps fastcdr)


add_executable(dds-perf-server
        ${IDL_SRCS}
        perf-server.cpp)

target_link_libraries(dds-perf-server fastrtps fastcdr)

add_executable(dds-overhead-client
        ${IDL_SRCS}
        overhead-client.cpp)
target_link_libraries(dds-overhead-client fastrtps fastcdr)


add_executable(dds-overhead-server
        ${IDL_SRCS}
        overhead-server.cpp)

target_link_libraries(dds-overhead-server fastrtps fastcdr)


add_executable(dds-multi-server
        ${IDL_SRCS}
        multi-server.cpp)

target_link_libraries(dds-multi-server fastrtps fastcdr)


add_executable(dds-multi-client
        ${IDL_SRCS}
        multi-client.cpp)

target_link_libraries(dds-multi-client fastrtps fastcdr)