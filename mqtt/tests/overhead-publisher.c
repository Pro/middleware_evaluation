
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <math.h>
#include <unistd.h>
#include "MQTTClient.h"

#define CLIENTID    "O"
#define TOPIC_DATA       "s"

#define QOS         0
#define TIMEOUT     10000L

volatile MQTTClient_deliveryToken deliveredtoken;
void delivered(void *context, MQTTClient_deliveryToken dt)
{
    //printf("Message with token value %d delivery confirmed\n", dt);
    deliveredtoken = dt;
}

void connlost(void *context, char *cause)
{
    printf("\nConnection lost\n");
    printf("     cause: %s\n", cause);
}


int main(int argc, char *argv[]) {

    if (argc != 3) {
        printf("Usage: program <ENDPOINT_URL> <PAYLOAD_BYTES>\nEnpoint URL is normally 'tcp://localhost:1883'\n");
        return 1;
    }

    int payloadSize = atoi(argv[2]);

    if (payloadSize < 0){
        printf("Payload size cannot be negative\n");
        return 1;
    }

    printf("Peparing payload of %d bytes to %s\n", payloadSize, argv[1]);


    uint8_t *data = malloc(payloadSize);

    if (data == NULL) {
        printf("Cannot alloc byte buffer\n");
        return 1;
    }

    srand((unsigned int) time(NULL));
    for (int i=0; i<payloadSize; i++)
        data[i] = (uint8_t)(rand()%255);

    MQTTClient client;
    MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;

    int rc;
    MQTTClient_create(&client, argv[1], CLIENTID,
                      MQTTCLIENT_PERSISTENCE_NONE, NULL);
    conn_opts.keepAliveInterval = 20;
    conn_opts.cleansession = 1;
    MQTTClient_setCallbacks(client, NULL, connlost, NULL, delivered);
    if ((rc = MQTTClient_connect(client, &conn_opts)) != MQTTCLIENT_SUCCESS)
    {
        printf("Failed to connect, return code %d\n", rc);
        exit(EXIT_FAILURE);
    }


    MQTTClient_deliveryToken token;


    MQTTClient_message pubmsg = MQTTClient_message_initializer;

    pubmsg.payload = data;
    pubmsg.payloadlen = (int)payloadSize;
    pubmsg.qos = QOS;
    pubmsg.retained = 0;


    printf("Connected. Sleeping a bit...\n");
    sleep(5);

    printf("Publish 1\n");
    if (MQTTClient_publishMessage(client, TOPIC_DATA, &pubmsg, &token) != MQTTCLIENT_SUCCESS) {
        fprintf(stderr, "Could not publish echo data");
        return false;
    }

    sleep(2);


    printf("Publish 1\n");
    if (MQTTClient_publishMessage(client, TOPIC_DATA, &pubmsg, &token) != MQTTCLIENT_SUCCESS) {
        fprintf(stderr, "Could not publish echo data");
        return false;
    }

    printf("All published. Sleeping a bit...\n");
    sleep(5);

    // bye bye
    MQTTClient_disconnect(client, 10000);
    MQTTClient_destroy(&client);
    return rc;
}
