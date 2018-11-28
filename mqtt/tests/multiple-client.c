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



#define PAYLOAD_SIZE 10240
#define PARALLEL_FORWARD 10
#define RUNS 100

bool running = true;
#define QOS         0

uint64_t get_microseconds() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
    return ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
}

char *topicSelf;
uint64_t dataReceived = 0;

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
    //printf("Got data %ld\n", dataReceived);
    if (strcmp(topicName, topicSelf) == 0) {
        dataReceived += message->payloadlen;
    }
    MQTTClient_freeMessage(&message);
    MQTTClient_free(topicName);
    return 1;
}


int main(int argc, char **argv)
{

    MQTTClient client;
    signal(SIGINT, stopHandler);
    signal(SIGTERM, stopHandler);

    if (argc != 5) {
        printf("Usage: program MQTT_BROKER TOPIC_SELF TOPIC_FORWARD_PREFIX TOPIC_FORWARD_IDX\n");
        return 1;
    }

    char *mqttBroker = argv[1];
    topicSelf = argv[2];
    char *topicForwardPrefix = argv[3];
    int topicForwardStart = atoi(argv[4]);

    char payload[PAYLOAD_SIZE];
    memset(payload, '*', PAYLOAD_SIZE);

    MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;

    char clientId[255];
    snprintf(clientId, 255, "client_%s", topicSelf);

    int rc;
    MQTTClient_create(&client, mqttBroker, clientId,
                      MQTTCLIENT_PERSISTENCE_NONE, NULL);
    conn_opts.keepAliveInterval = 20;
    conn_opts.cleansession = 1;
    MQTTClient_setCallbacks(client, NULL, connlost, msgarrvd, delivered);
    printf("Connecting to Broker: %s\n", mqttBroker);
    if ((rc = MQTTClient_connect(client, &conn_opts)) != MQTTCLIENT_SUCCESS)
    {
        printf("Failed to connect, return code %d\n", rc);
        exit(EXIT_FAILURE);
    }
    MQTTClient_subscribe(client, topicSelf, QOS);

    char topicNames[PARALLEL_FORWARD][255];
    for (unsigned int i = 0; i < PARALLEL_FORWARD; i++) {
        snprintf(topicNames[i], 255, "%s%d", topicForwardPrefix, topicForwardStart+i);

    }
    MQTTClient_message pubmsg = MQTTClient_message_initializer;
    pubmsg.payload = payload;
    pubmsg.payloadlen = PAYLOAD_SIZE;
    pubmsg.qos = QOS;
    pubmsg.retained = 0;
    MQTTClient_deliveryToken token;

    uint64_t elapsedTime[RUNS];
    for (unsigned int k = 0; k < RUNS && running; k++) {
        dataReceived = 0;


        printf("[%d/%d] Publishing data\n", k, RUNS);
        uint64_t start_time = get_microseconds();
        for (unsigned int i = 0; i < PARALLEL_FORWARD; i++) {


            if (MQTTClient_publishMessage(client, topicNames[i], &pubmsg, &token) != MQTTCLIENT_SUCCESS) {
                fprintf(stderr, "Could not publish echo data\n");
                return false;
            }
        }

        while (running && dataReceived != PARALLEL_FORWARD * PAYLOAD_SIZE) {
            MQTTClient_yield();
            usleep(10);
        }
        elapsedTime[k] = get_microseconds() - start_time;
        printf("[%d/%d] All data received.\n", k, RUNS);
    }

    if (running) {

        uint64_t totalTime = 0;
        printf("-------------\nNode;Microseconds\n");
        for (unsigned int k = 0; k < RUNS; k++) {
            printf("%d;%ld\n",k, elapsedTime[k]);
            if (k > 0) // skip first, since it also includes setting up connection
                totalTime += elapsedTime[k];
        }
        printf("---------------\n");
        printf("Total Elapsed = %fms, Average = %fms\n", totalTime/1000.0, (totalTime / (double)(RUNS-1))/1000.0);

    }

    // bye bye
    MQTTClient_disconnect(client, 10000);
    MQTTClient_destroy(&client);

    return 0;
}