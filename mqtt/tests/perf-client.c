
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <math.h>
#include <unistd.h>
#include "MQTTClient.h"
#include "time_utils.h"
#include "test_wrapper.h"

#define ADDRESS     "tcp://cobot-t1-main:1883"
#define CLIENTID    "ExampleClientPub"
#define TOPIC_ECHO_SEND       "mqtt_echo_send"
#define TOPIC_ECHO_REC       "mqtt_echo_rec"
#define TOPIC_ACK_SEND       "mqtt_ack_send"
#define TOPIC_ACK_REC       "mqtt_ack_rec"

#define QOS         0

uint16_t currentMessageId = 0;
uint16_t receivedMessageId = 0;

volatile MQTTClient_deliveryToken deliveredtoken;
void delivered(void *context, MQTTClient_deliveryToken dt)
{
    //printf("Message with token value %d delivery confirmed\n", dt);
    deliveredtoken = dt;
}

static bool
perform_method_call(void *context, void *payloadData, size_t payloadSize, long long int *elapsedTime, bool isEcho){
    MQTTClient *client = (MQTTClient*)context;
    char *arg= (char*)payloadData;
    size_t argLen = payloadSize;

    currentMessageId++;

    MQTTClient_deliveryToken token;


    MQTTClient_message pubmsg = MQTTClient_message_initializer;

    // set the first two bytes of the payload to topic id
    if (argLen < 2)
        return false;
    *((uint16_t*)arg) = currentMessageId;
    pubmsg.payload = arg;
    pubmsg.payloadlen = (int)argLen;
    pubmsg.qos = QOS;
    pubmsg.retained = 0;

    struct timespec time_start;
    TIME_MEASURE_START(time_start);
    if (isEcho) {
        if (MQTTClient_publishMessage(client, TOPIC_ECHO_SEND, &pubmsg, &token) != MQTTCLIENT_SUCCESS) {
            fprintf(stderr, "Could not publish echo data");
            return false;
        }
    }
    else {
        if (MQTTClient_publishMessage(client, TOPIC_ACK_SEND, &pubmsg, &token) != MQTTCLIENT_SUCCESS) {
            fprintf(stderr, "Could not publish echo data");
            return false;
        }
    }
    while (receivedMessageId != currentMessageId) {
        usleep(1);
    }
    TIME_MEASURE_DIFF_USEC(time_start, *elapsedTime);

    return true;
}


int msgarrvd(void *context, char *topicName, int topicLen, MQTTClient_message *message)
{
    /*printf("Message arrived\n");
    printf("     topic: %s\n", topicName);
    printf("   message: ");*/
    if (strcmp(topicName, TOPIC_ECHO_REC) == 0 || strcmp(topicName, TOPIC_ACK_REC) == 0) {
        if (message->payloadlen < 2) {
            fprintf(stderr, "Invalid message length for echo rec");
        } else {
            receivedMessageId = *((uint16_t*)message->payload);
        }
    }
    MQTTClient_freeMessage(&message);
    MQTTClient_free(topicName);
    return 1;
}
void connlost(void *context, char *cause)
{
    printf("\nConnection lost\n");
    printf("     cause: %s\n", cause);
}


int main(int argc, char *argv[]) {
    // intialize the large string that will be used for the function calls and set every char to '*'
    char mainString[(int)pow(2,max_message_size_pow)];
    for(int i = 0; i < sizeof(mainString); i++){mainString[i] = '*';}
    // init. the struct that will be sent to the server, and the variable that will hold the elapsed time for each call

    MQTTClient client;
    MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;

    int rc;
    MQTTClient_create(&client, ADDRESS, CLIENTID,
                      MQTTCLIENT_PERSISTENCE_NONE, NULL);
    conn_opts.keepAliveInterval = 20;
    conn_opts.cleansession = 1;
    MQTTClient_setCallbacks(client, NULL, connlost, msgarrvd, delivered);
    if ((rc = MQTTClient_connect(client, &conn_opts)) != MQTTCLIENT_SUCCESS)
    {
        printf("Failed to connect, return code %d\n", rc);
        exit(EXIT_FAILURE);
    }
    MQTTClient_subscribe(client, TOPIC_ACK_REC, QOS);
    MQTTClient_subscribe(client, TOPIC_ECHO_REC, QOS);

    /* For printing progress in a single line with dots */
    setbuf(stdout, NULL);


    bool success = runTests("mqtt", &perform_method_call, client, mainString);

    // bye bye
    MQTTClient_disconnect(client, 10000);
    MQTTClient_destroy(&client);
    return rc;
}
