//
// Created by profanter on 8/29/18.
// Copyright (c) 2018 fortiss GmbH. All rights reserved.
//

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include "ros_tests/Array.h"

ros::Publisher *pubForward;

void forwardCallback(const ros_tests::Array::ConstPtr& msg)
{
    pubForward->publish(msg);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "multi_forward_node");

    ros::NodeHandle nh("~");

    std::string topic_self, topic_forward;
    if (!nh.getParam("topic_self", topic_self))
    {
        ROS_ERROR("Did not find parameter for 'topic_self");
        return 1;
    }
    if (!nh.getParam("topic_forward", topic_forward))
    {
        ROS_ERROR("Did not find parameter for 'topic_forward");
        return 1;
    }

    ros::Subscriber sub = nh.subscribe(topic_self, 1, forwardCallback);


    ros::Publisher pub = nh.advertise<ros_tests::Array>(topic_forward, 1, true);
    pubForward = &pub;

    ros::Rate loop_rate(100);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}