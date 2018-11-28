cmake_minimum_required(VERSION 2.8.4)
project("MQTT Test" C)

add_executable(mqtt-perf-client perf-client.c)
target_link_libraries(mqtt-perf-client paho-mqtt3c pthread m)

add_executable(mqtt-perf-server perf-server.c)
target_link_libraries(mqtt-perf-server paho-mqtt3c pthread)

add_executable(mqtt-test-overhead overhead-publisher.c)
target_link_libraries(mqtt-test-overhead paho-mqtt3c pthread)

add_executable(mqtt-multi-server multiple-server.c)
target_link_libraries(mqtt-multi-server paho-mqtt3c pthread)

add_executable(mqtt-multi-client multiple-client.c)
target_link_libraries(mqtt-multi-client paho-mqtt3c pthread)
