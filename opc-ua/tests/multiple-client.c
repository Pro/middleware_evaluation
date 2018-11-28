//
// Created by profanter on 8/29/18.
// Copyright (c) 2018 fortiss GmbH. All rights reserved.
//

#include <signal.h>
#include <stdio.h>
#include <string.h>
#include "open62541.h"
#include <pthread.h>
#include <unistd.h>

UA_Boolean running = true;

static void stopHandler(int sig) {
    UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_USERLAND, "received ctrl-c");
    running = false;
}

UA_UInt64 dataReceived = 0;

pthread_t server_thread;
UA_Server *server;


uint64_t get_microseconds() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
    return ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
}


#define PAYLOAD_SIZE 10240
#define PARALLEL_FORWARD 10
#define RUNS 100

static void *serverloop(void *_) {
    while (running)
        UA_Server_run_iterate(server, true);
    return 0;
}

UA_ByteString *payloadSend;

static UA_StatusCode
readDataFromForward(UA_Server *server, const UA_NodeId *sessionId,
                    void *sessionContext, const UA_NodeId *nodeId,
                    void *nodeContext, UA_Boolean includeSourceTimeStamp,
                    const UA_NumericRange *range, UA_DataValue *value) {

    UA_ByteString bytes = UA_BYTESTRING_NULL;
    UA_Variant_setScalar(&value->value, &bytes, &UA_TYPES[UA_TYPES_BYTESTRING]);
    return UA_STATUSCODE_GOOD;
}

static UA_StatusCode
writeDataToForward(UA_Server *server,
                   const UA_NodeId *sessionId, void *sessionContext,
                   const UA_NodeId *nodeId, void *nodeContext,
                   const UA_NumericRange *range, const UA_DataValue *data) {
    if (!data->hasValue) {
        UA_LOG_ERROR(UA_Log_Stdout, UA_LOGCATEGORY_USERLAND,
                     "Got empty data value.");
        return UA_STATUSCODE_BADINTERNALERROR;
    }
    if (data->value.type != &UA_TYPES[UA_TYPES_BYTESTRING]) {
        UA_LOG_ERROR(UA_Log_Stdout, UA_LOGCATEGORY_USERLAND,
                     "Expected ByteString data value.");
        return UA_STATUSCODE_BADINTERNALERROR;
    }

    UA_ByteString *payload = (UA_ByteString *) data->value.data;

    dataReceived += payload->length;

    /*UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_USERLAND,
                "Received %ld bytes. Total = %ld", payload->length, dataReceived);*/


    return UA_STATUSCODE_GOOD;
}

static void
addForwardNode(UA_Server *server) {
    UA_VariableAttributes attr = UA_VariableAttributes_default;
    attr.displayName = UA_LOCALIZEDTEXT("en-US", "ForwardData");
    attr.accessLevel = UA_ACCESSLEVELMASK_READ | UA_ACCESSLEVELMASK_WRITE;

    UA_DataSource forwardDataSource;
    forwardDataSource.read = readDataFromForward;
    forwardDataSource.write = writeDataToForward;
    UA_StatusCode retval = UA_Server_addDataSourceVariableNode(server,
                                                               UA_NODEID_NUMERIC(1, 1000),
                                                               UA_NODEID_NUMERIC(0, UA_NS0ID_OBJECTSFOLDER),
                                                               UA_NODEID_NUMERIC(0, UA_NS0ID_ORGANIZES),
                                                               UA_QUALIFIEDNAME(1, "ForwardData"),
                                                               UA_NODEID_NUMERIC(0, UA_NS0ID_BASEDATAVARIABLETYPE),
                                                               attr,
                                                               forwardDataSource, NULL, NULL);

    if (retval != UA_STATUSCODE_GOOD) {
        UA_LOG_ERROR(UA_Log_Stdout, UA_LOGCATEGORY_SERVER, "Cannot create variable node. %s",
                     UA_StatusCode_name(retval));
    }
}

struct ClientEndpointData {
    UA_Client *client;
    char endpoint[255];
};

static void *clientLoop(void *data) {

    struct ClientEndpointData *d = (struct ClientEndpointData *) data;


    if (UA_Client_getState(d->client) != UA_CLIENTSTATE_CONNECTED) {

        UA_StatusCode retval = UA_Client_connect(d->client, d->endpoint);
        if (retval != UA_STATUSCODE_GOOD) {
            UA_LOG_ERROR(UA_Log_Stdout, UA_LOGCATEGORY_CLIENT, "Can not connect to endpoint %s", d->endpoint);
            return NULL;
        }

    }

    UA_Variant var;
    UA_Variant_setScalar(&var, payloadSend, &UA_TYPES[UA_TYPES_BYTESTRING]);


    UA_StatusCode retval = UA_Client_writeValueAttribute(d->client, UA_NODEID_NUMERIC(1, 1000),
                                                         &var);
    if (retval != UA_STATUSCODE_GOOD) {
        UA_LOG_ERROR(UA_Log_Stdout, UA_LOGCATEGORY_CLIENT, "Can not write to endpoint %s: %s", d->endpoint,
                     UA_StatusCode_name(retval));
        return NULL;
    }
    return 0;
}


// -----------------------------------------------------------------------------------------
int main(int argc, char *argv[]) {
    signal(SIGINT, stopHandler);
    signal(SIGTERM, stopHandler);

    if (argc != 4) {
        UA_LOG_ERROR(UA_Log_Stdout, UA_LOGCATEGORY_USERLAND,
                     "Usage: program SERVER_PORT CLIENT_START_PORT FORWARD_ENDPOINT");
        return 1;
    }

    UA_UInt16 server_port = atoi(argv[1]);
    if (server_port == 0) {
        UA_LOG_ERROR(UA_Log_Stdout, UA_LOGCATEGORY_USERLAND, "Invalid Server Port");
        return 1;
    }

    UA_UInt16 start_port = atoi(argv[2]);
    if (start_port == 0) {
        UA_LOG_ERROR(UA_Log_Stdout, UA_LOGCATEGORY_USERLAND, "Invalid Client Port");
        return 1;
    }

    char *start_endpoint = argv[3];

    UA_ServerConfig *config = UA_ServerConfig_new_minimal(server_port, NULL);
    server = UA_Server_new(config);

    addForwardNode(server);

    UA_Byte dataArr[PAYLOAD_SIZE];
    memset(dataArr, '*', PAYLOAD_SIZE);

    UA_ByteString payload;
    payloadSend = &payload;

    payloadSend->data = dataArr;
    payloadSend->length = PAYLOAD_SIZE;


    pthread_t clientThreads[PARALLEL_FORWARD];
    struct ClientEndpointData clientData[PARALLEL_FORWARD];

    UA_Server_run_startup(server);

    pthread_create(&server_thread, NULL, &serverloop, NULL);

    for (unsigned int i = 0; i < PARALLEL_FORWARD; i++) {


        snprintf(clientData[i].endpoint, 255, "%s:%d", start_endpoint, start_port + i);
        clientData[i].client = UA_Client_new(UA_ClientConfig_default);
    }

    uint64_t elapsedTime[RUNS];

    for (unsigned int k = 0; k < RUNS; k++) {
        dataReceived = 0;

        UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_USERLAND, "[%d/%d] Publishing data", k, RUNS);
        uint64_t start_time = get_microseconds();
        for (unsigned int i = 0; i < PARALLEL_FORWARD; i++) {
            pthread_create(&clientThreads[i], NULL, &clientLoop, &clientData[i]);
        }
        for (unsigned int i = 0; i < PARALLEL_FORWARD; i++) {
            pthread_join(clientThreads[i], NULL);
        }

        while (dataReceived != PARALLEL_FORWARD * payloadSend->length) {
            usleep(10);
        }
        elapsedTime[k] = get_microseconds() - start_time;
        UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_USERLAND, "[%d/%d] All data received.", k, RUNS);
    }

    for (unsigned int i = 0; i < PARALLEL_FORWARD; i++) {
        UA_Client_disconnect(clientData[i].client);
        UA_Client_delete(clientData[i].client);
    }

    uint64_t totalTime = 0;
    printf("-------------\nNode;Microseconds\n");
    for (unsigned int k = 0; k < RUNS; k++) {
        printf("%d;%ld\n",k, elapsedTime[k]);
        if (k > 0) // skip first, since it also includes setting up connection
            totalTime += elapsedTime[k];
    }
    printf("---------------\n");
    UA_LOG_WARNING(UA_Log_Stdout, UA_LOGCATEGORY_USERLAND, "Total Elapsed = %fms, Average = %fms", totalTime/1000.0, (totalTime / (double)(RUNS-1))/1000.0);



    running = false;

    pthread_join(server_thread, NULL);

    UA_Server_delete(server);
    UA_ServerConfig_delete(config);
    return (int) 0;
}
