#include "ros/ros.h"
#include "ros_tests/AckStr.h"
#include "ros_tests/EchoStr.h"

#include "proc_stat.h"

bool ack(ros_tests::AckStr::Request &req,
         ros_tests::AckStr::Response &res) {
    res.ack = true;
    //ROS_INFO("request: x=%s", req.a);
    //ROS_INFO("  sending back response: [%s]", res.a_out);
    return true;
}


bool echo(ros_tests::EchoStr::Request &req,
          ros_tests::EchoStr::Response &res) {
    res.a_out = req.a;
    //ROS_INFO("request: x=%s", req.a);
    //ROS_INFO("  sending back response: [%s]", res.a_out);
    return true;
}


int main(int argc, char **argv) {


    char procStatFile[255];
    struct tm *timenow;
    time_t now = time(NULL);
    timenow = gmtime(&now);
    strftime(procStatFile, 255, "procStat-ros-%Y-%m-%d_%H%M%S.csv",timenow);


    int procStatPid = runProcStatToCsv(procStatFile);
    if (procStatPid <= 0)
        return 1;

    ros::init(argc, argv, "ack_str_server");
    ros::NodeHandle n;

// %Tag(SERVICE_SERVER)%
    ros::ServiceServer service_ack = n.advertiseService("ack_str", ack);
    ros::ServiceServer service_str = n.advertiseService("echo_str", echo);
// %EndTag(SERVICE_SERVER)%

    ROS_INFO("Node running");

    ros::spin();


    stopProcToCsv(procStatPid);

    printf("Process Status written to: %s\n", procStatFile);


    return 0;
}

