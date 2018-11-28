#include "ros/ros.h"
#include "ros_tests/AckStr.h"
#include "ros_tests/EchoStr.h"
#include <cstdlib>
#include <sys/time.h>
#include <math.h>
#include <iostream>
#include <string>
#include <sstream>

#include <time_utils.h>
#include <test_wrapper.h>

// whether to maintain connection between seperate calls
#define maintain_connection true

// -Method to expand a stringstream to be able to get variable sized strings
void expand_str(std::stringstream &stream, int len) {
    std::string c(len, '*');
    stream << c;
}
// ---------------------------------------------------

ros::ServiceClient *client_es;
ros::ServiceClient *client_as;

static bool
perform_method_call(void *context, void *payloadData, size_t payloadSize, long long int *elapsedTime, bool isEcho) {

    unsigned char *payloadStr = (unsigned char*) payloadData;

    if (isEcho) {
        ros_tests::EchoStr srv;
        srv.request.a.reserve(payloadSize);
        for (size_t i=0; i<payloadSize; i++)
            srv.request.a.push_back(payloadStr[i]);

        struct timespec time_start;
        TIME_MEASURE_START(time_start);
        if (client_es->call(srv)) {
            TIME_MEASURE_DIFF_USEC(time_start, *elapsedTime);
        } else {
            ROS_ERROR("ERROR: Failed to call service ack_str");
            return false;
        }
    } else {
        ros_tests::AckStr srv;
        srv.request.a.reserve(payloadSize);
        for (size_t i=0; i<payloadSize; i++)
            srv.request.a.push_back(payloadStr[i]);

        struct timespec time_start;
        TIME_MEASURE_START(time_start);
        if (client_as->call(srv)) {
            TIME_MEASURE_DIFF_USEC(time_start, *elapsedTime);
        } else {
            ROS_ERROR("ERROR: Failed to call service ack_str");
            return false;
        }
    }
    return true;
}

int main(int argc, char **argv) {
    // Init ROS
    ros::init(argc, argv, "testing_client");
    if (argc != 1) {
        ROS_INFO("usage: testing_client");
        return 1;
    }
    ros::NodeHandle n;
    // -------------------------------------------------

    // Init the service client for echo_str
    ros::ServiceClient c_es = n.serviceClient<ros_tests::EchoStr>("echo_str", maintain_connection);
    client_es = &c_es;
    ros_tests::EchoStr srv_es;
    // -------------------------------------------------

    // Init the service client for ack_str
    ros::ServiceClient c_as = n.serviceClient<ros_tests::AckStr>("ack_str", maintain_connection);
    client_as = &c_as;
    ros_tests::AckStr srv_as;
    // -------------------------------------------------

    // For printing progress in a single line with dots
    setbuf(stdout, NULL);
    // -------------------------------------------------

    unsigned char mainString[(int)pow(2,max_message_size_pow)];
    memset(mainString, sizeof(mainString), '*');

    bool success = runTests("ros", &perform_method_call, NULL, &mainString);

    return 0;
}
