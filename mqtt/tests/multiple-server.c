//
// Created by profanter on 8/31/18.
// Copyright (c) 2018 fortiss GmbH. All rights reserved.
//


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <math.h>
#include <unistd.h>
#include <signal.h>
#include "MQTTClient.h"

bool running = true;
#define QOS         0

char *mqttBroker, *topicSelf, *topicForward;
MQTTClient client;

static void stopHandler(int sig) {
    printf("received ctrl-c\n");
    running = false;
}

void connlost(void *context, char *cause)
{
    printf("\nConnection lost\n");
    printf("     cause: %s\n", cause);
}


volatile MQTTClient_deliveryToken deliveredtoken;
void delivered(void *context, MQTTClient_deliveryToken dt)
{
    //printf("Message with token value %d delivery confirmed\n", dt);
    deliveredtoken = dt;
}

int msgarrvd(void *context, char *topicName, int topicLen, MQTTClient_message *message)
{
    MQTTClient_deliveryToken token;
    if (strcmp(topicName, topicSelf) == 0) {
        MQTTClient_message pubmsg = MQTTClient_message_initializer;
        pubmsg.payload = message->payload;
        pubmsg.payloadlen = message->payloadlen;
        pubmsg.qos = QOS;
        pubmsg.retained = 0;

        int ret = MQTTClient_publishMessage(client, topicForward, &pubmsg, &token);
        if (ret != MQTTCLIENT_SUCCESS) {
            fprintf(stderr, "Could not publish forward data. Error: %d\n", ret);
        }
    }
    MQTTClient_freeMessage(&message);
    MQTTClient_free(topicName);
    return 1;
}


int main(int argc, char **argv)
{

    signal(SIGINT, stopHandler);
    signal(SIGTERM, stopHandler);

    if (argc != 4) {
        printf("Usage: program MQTT_BROKER TOPIC_SELF TOPIC_FORWARD\n");
        return 1;
    }

    mqttBroker = argv[1];
    topicSelf = argv[2];
    topicForward = argv[3];

    MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;

    char clientId[255];
    snprintf(clientId, 255, "client_%s", topicSelf);

    int rc;
    MQTTClient_create(&client, mqttBroker, clientId,
                      MQTTCLIENT_PERSISTENCE_NONE, NULL);
    conn_opts.keepAliveInterval = 20;
    conn_opts.cleansession = 1;
    MQTTClient_setCallbacks(client, NULL, connlost, msgarrvd, delivered);
    if ((rc = MQTTClient_connect(client, &conn_opts)) != MQTTCLIENT_SUCCESS)
    {
        printf("Failed to connect, return code %d\n", rc);
        exit(EXIT_FAILURE);
    }
    MQTTClient_subscribe(client, topicSelf, QOS);

    printf("Forwarder ready: %s -> %s\n", topicSelf, topicForward);

    while(running) {
        MQTTClient_yield();
        usleep(1000);
    }

    // bye bye
    MQTTClient_disconnect(client, 10000);
    MQTTClient_destroy(&client);

    return 0;
}