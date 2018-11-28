//
// Created by profanter on 8/29/18.
// Copyright (c) 2018 fortiss GmbH. All rights reserved.
//

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include "ros_tests/Array.h"
#include <string>
#include <iostream>

#define PAYLOAD_SIZE 10240
#define PARALLEL_FORWARD 10
#define RUNS 100

ros_tests::Array payloadSend;
uint64_t dataReceived = 0;

uint64_t get_microseconds() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
    return ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
}

static void *clientLoop(void *data) {
    ros::Publisher *pub = (ros::Publisher *)data;

}

void forwardCallback(const ros_tests::Array::ConstPtr& msg)
{
    dataReceived += msg->data.size();
}


int main(int argc, char **argv)
{
    srand(time(NULL));
    std::string nodeName = "multi_forward_client";
    ros::init(argc, argv, nodeName);

    ros::NodeHandle nh("~");

    std::string topic_self, topic_forward;
    int topic_forward_idx;
    if (!nh.getParam("topic_self", topic_self))
    {
        ROS_ERROR("Did not find parameter for 'topic_self");
        return 1;
    }
    if (!nh.getParam("topic_forward_prefix", topic_forward))
    {
        ROS_ERROR("Did not find parameter for 'topic_forward_prefix");
        return 1;
    }
    if (!nh.getParam("topic_forward_idx", topic_forward_idx))
    {
        ROS_ERROR("Did not find parameter for 'topic_forward_idx");
        return 1;
    }

    ros_tests::Array payload;
    payload.data.reserve(PAYLOAD_SIZE);
    srand((unsigned int) time(NULL));
    for (int i=0; i<PAYLOAD_SIZE; i++)
        payload.data.push_back((uint8_t)(rand()%255));

    ros::Subscriber sub = nh.subscribe(topic_self, PARALLEL_FORWARD*2, forwardCallback);
    ros::Publisher pubs[PARALLEL_FORWARD];

    for (unsigned int i = 0; i < PARALLEL_FORWARD; i++) {
	std::ostringstream ss;
	ss << (topic_forward_idx + i);
        std::string pubTopic = topic_forward + ss.str();
        pubs[i] = nh.advertise<ros_tests::Array>(pubTopic, 1, true);
    }

    uint64_t elapsedTime[RUNS];
    for (unsigned int k = 0; k < RUNS && ros::ok(); k++) {
        dataReceived = 0;



        ROS_INFO("[%d/%d] Publishing data", k, RUNS);
        uint64_t start_time = get_microseconds();
        for (unsigned int i = 0; i < PARALLEL_FORWARD; i++) {
            pubs[i].publish(payload);
        }

        while (ros::ok() && dataReceived != PARALLEL_FORWARD * PAYLOAD_SIZE) {
            ros::spinOnce();
            usleep(10);
        }
        elapsedTime[k] = get_microseconds() - start_time;
        ROS_INFO("[%d/%d] All data received.", k, RUNS);
    }

    uint64_t totalTime = 0;
    printf("-------------\nNode;Microseconds\n");
    for (unsigned int k = 0; k < RUNS; k++) {
        printf("%d;%ld\n",k, elapsedTime[k]);
        if (k > 0) // skip first, since it also includes setting up connection
            totalTime += elapsedTime[k];
    }
    printf("---------------\n");
    ROS_INFO("Total Elapsed = %fms, Average = %fms", totalTime/1000.0, (totalTime / (double)(RUNS-1))/1000.0);


    return 0;
}
