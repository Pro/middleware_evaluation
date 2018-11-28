cmake_minimum_required(VERSION 2.8.4)
project("DDS Test" C)


SET(IDL_SRCS
        idl/DDSPerfTestC.cpp
        idl/DDSPerfTestS.cpp
        idl/DDSPerfTestTypeSupportC.cpp
        idl/DDSPerfTestTypeSupportS.cpp
        idl/DDSPerfTestTypeSupportImpl.cpp
        )
include_directories(idl)

#add_executable(dds-overhead-pub
#        ${IDL_SRCS}
#        Writer.cpp
#        overhead-pub.cpp)
#target_link_libraries(dds-overhead-pub ${OpenDDS_LIBRARIES})
#
#add_executable(dds-overhead-sub
#        ${IDL_SRCS}
#        overhead-sub.cpp)
#
#target_link_libraries(dds-overhead-sub ${OpenDDS_LIBRARIES})

add_executable(dds-perf-client
        ${IDL_SRCS}
        AckDataSender.cpp
        EchoDataSender.cpp
        perf-client.cpp)
target_link_libraries(dds-perf-client ${OpenDDS_LIBRARIES})


add_executable(dds-perf-server
        ${IDL_SRCS}
        AckDataReply.cpp
        EchoDataReply.cpp
        perf-server.cpp)

target_link_libraries(dds-perf-server ${OpenDDS_LIBRARIES})


add_executable(dds-multi-server
        ${IDL_SRCS}
        EchoDataReply.cpp
        multi-server.cpp)

target_link_libraries(dds-multi-server ${OpenDDS_LIBRARIES})


add_executable(dds-multi-client
        ${IDL_SRCS}
        EchoDataReader.cpp
        multi-client.cpp)

target_link_libraries(dds-multi-client ${OpenDDS_LIBRARIES})