//
// Created by profanter on 8/23/18.
// Copyright (c) 2018 fortiss GmbH. All rights reserved.
//

#include <stdio.h>
#include <stdlib.h>

#include "ros/ros.h"

#include "ros_tests/Array.h"


int main(int argc, char **argv)
{


    ros::init(argc, argv, "OverheadPublisher");

    ros::NodeHandle n("~");

    int payloadSize;

    n.getParam("payload", payloadSize);

    ros::Publisher pub = n.advertise<ros_tests::Array>("/data", 1, true);
    ros::spinOnce();


    ros_tests::Array arr;

    arr.data.reserve(payloadSize);

    sleep(5);


    srand((unsigned int) time(NULL));
    for (int i=0; i<payloadSize; i++)
        arr.data.push_back((uint8_t)(rand()%255));

    pub.publish(arr);
    ros::spinOnce();


    sleep(2);
    pub.publish(arr);
    ros::spinOnce();
    sleep(5);


}