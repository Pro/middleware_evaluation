
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <math.h>
#include <signal.h>
#include <unistd.h>
#include "MQTTClient.h"


#include "proc_stat.h"

#define ADDRESS     "tcp://10.200.2.62:1883"
#define CLIENTID    "ExampleServerEcho"
#define TOPIC_ECHO_SEND       "mqtt_echo_send"
#define TOPIC_ECHO_REC       "mqtt_echo_rec"
#define TOPIC_ACK_SEND       "mqtt_ack_send"
#define TOPIC_ACK_REC       "mqtt_ack_rec"

#define QOS         0

MQTTClient client;


int msgarrvd(void *context, char *topicName, int topicLen, MQTTClient_message *message)
{
    /*printf("Message arrived\n");
    printf("     topic: %s\n", topicName);
    printf("   message: ");*/
    if (strcmp(topicName, TOPIC_ECHO_SEND) == 0) {

        // send back payload
        MQTTClient_message pubmsg = MQTTClient_message_initializer;

        pubmsg.payload = message->payload;
        pubmsg.payloadlen = message->payloadlen;
        pubmsg.qos = QOS;
        pubmsg.retained = 0;

        if (MQTTClient_publishMessage(client, TOPIC_ECHO_REC, &pubmsg, NULL) != MQTTCLIENT_SUCCESS) {
            fprintf(stderr, "Could not publish echo data");
            return false;
        }

        //printf("Confirm echo message %d", (uint16_t)(*(uint16_t*)message->payload));
    } else if (strcmp(topicName, TOPIC_ACK_SEND) == 0) {

        MQTTClient_message pubmsg = MQTTClient_message_initializer;

        // only send back first two bytes since these include the topic id
        pubmsg.payload = message->payload;
        pubmsg.payloadlen = sizeof(uint16_t);
        pubmsg.qos = QOS;
        pubmsg.retained = 0;

        if (MQTTClient_publishMessage(client, TOPIC_ACK_REC, &pubmsg, NULL) != MQTTCLIENT_SUCCESS) {
            fprintf(stderr, "Could not publish ack data");
            return false;
        }
        //printf("Confirm ack message %d", (uint16_t)(*(uint16_t*)message->payload));
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


bool running = true;
static void stopHandler(int sig) {
    running = false;
}


int main(int argc, char *argv[]) {


    signal(SIGINT, stopHandler);
    signal(SIGTERM, stopHandler);

    setProcPriority();

    char procStatFile[255];
    struct tm *timenow;
    time_t now = time(NULL);
    timenow = gmtime(&now);
    strftime(procStatFile, 255, "procStat-mqtt-%Y-%m-%d_%H%M%S.csv",timenow);


    int procStatPid = runProcStatToCsv(procStatFile);
    if (procStatPid <= 0)
        return 1;

    MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;

    int rc;
    MQTTClient_create(&client, ADDRESS, CLIENTID,
                      MQTTCLIENT_PERSISTENCE_NONE, NULL);
    conn_opts.keepAliveInterval = 20;
    conn_opts.cleansession = 1;
    MQTTClient_setCallbacks(client, NULL, connlost, msgarrvd, NULL);
    if ((rc = MQTTClient_connect(client, &conn_opts)) != MQTTCLIENT_SUCCESS)
    {
        printf("Failed to connect, return code %d\n", rc);
        exit(EXIT_FAILURE);
    }
    MQTTClient_subscribe(client, TOPIC_ACK_SEND, QOS);
    MQTTClient_subscribe(client, TOPIC_ECHO_SEND, QOS);
    printf("Echo server ready to receive MQTT messages\n");
    while(running)
        usleep(1000);

    // bye bye
    MQTTClient_disconnect(client, 10000);
    MQTTClient_destroy(&client);

    stopProcToCsv(procStatPid);

    printf("Process Status written to: %s\n", procStatFile);

    return rc;
}
