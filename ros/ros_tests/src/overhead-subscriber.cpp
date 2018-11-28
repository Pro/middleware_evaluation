//
// Created by profanter on 8/23/18.
// Copyright (c) 2018 fortiss GmbH. All rights reserved.
//

#include <stdio.h>
#include <stdlib.h>

#include "ros/ros.h"

#include "ros_tests/Array.h"

void arrayCallback(const ros_tests::Array& array)
{
    ROS_INFO("Got array with length: %ld", array.data.size());
    return;
}

int main(int argc, char **argv)
{


    ros::init(argc, argv, "OverheadSubscriber");

    ros::NodeHandle n("~");


    ros::Subscriber sub3 = n.subscribe("/data", 1, arrayCallback);

    ros::spin();

}