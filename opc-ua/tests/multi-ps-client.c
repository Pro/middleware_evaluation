//
// Created by ayhun on 01/02/18.
//
/* This work is licensed under a Creative Commons CCZero 1.0 Universal License.
 * See http://creativecommons.org/publicdomain/zero/1.0/ for more information. */
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <signal.h>
#include <time.h>
#define UA_NO_AMALGAMATION
#include "ua_pubsub_networkmessage.h"
#include "ua_log_stdout.h"
#include "ua_server.h"
#include "ua_config_default.h"
#include "ua_pubsub.h"
#include "ua_network_pubsub_udp.h"
#ifdef UA_ENABLE_PUBSUB_ETH_UADP
#include "ua_network_pubsub_ethernet.h"
#endif
#include "src_generated/ua_types_generated.h"
#include <stdio.h>
#include <signal.h>

#include <pthread.h>

struct UA_WriterGroup;
typedef struct UA_WriterGroup UA_WriterGroup;
void
UA_WriterGroup_publishCallback(UA_Server *server, UA_WriterGroup *writerGroup);

UA_WriterGroup *
UA_WriterGroup_findWGbyId(UA_Server *server, UA_NodeId identifier);

UA_NodeId subConnectionIdent;

#include "test_wrapper.h"

UA_NodeId payloadVariableNodeId = {
        .namespaceIndex = 1,
        .identifierType = UA_NODEIDTYPE_NUMERIC,
        .identifier.numeric = 50123
};


uint64_t get_microseconds() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
    return ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
}


#define PAYLOAD_SIZE 10240
#define PARALLEL_FORWARD 10
#define RUNS 100

UA_Boolean running = true;
static void stopHandler(int sign) {
    UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_SERVER, "received ctrl-c");
    running = false;
}

static void manuallyPublish(UA_Server *server, UA_NodeId writerGroupIdent) {
    UA_WriterGroup *wg = UA_WriterGroup_findWGbyId(server, writerGroupIdent);
    UA_WriterGroup_publishCallback(server, wg);
}

UA_UInt32 dataReceived = 0;


static void handleByteString(UA_Server *server, UA_ByteString* bs) {
    dataReceived += bs->length;
    //printf("Received %ld bytes\n", bs->length);
}


static void
subscriptionPollingCallback(UA_Server *server, UA_PubSubConnection *connection) {
    UA_ByteString buffer;
    if (UA_ByteString_allocBuffer(&buffer, (size_t)pow(2,max_message_size_pow)+100) != UA_STATUSCODE_GOOD) {
        UA_LOG_ERROR(UA_Log_Stdout, UA_LOGCATEGORY_SERVER,
                     "Message buffer allocation failed!");
        return;
    }

    /* Receive the message. Blocks for 1ms */
    UA_StatusCode retval =
            connection->channel->receive(connection->channel, &buffer, NULL, 1);
    if(retval != UA_STATUSCODE_GOOD || buffer.length == 0) {
        /* Workaround!! Reset buffer length. Receive can set the length to zero.
         * Then the buffer is not deleted because no memory allocation is
         * assumed.
         * TODO: Return an error code in 'receive' instead of setting the buf
         * length to zero. */
        buffer.length = 512;
        UA_ByteString_deleteMembers(&buffer);
        return;
    }

    /* Decode the message */
    /*UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_USERLAND,
                "Message length: %lu", (unsigned long) buffer.length);*/
    UA_NetworkMessage networkMessage;
    memset(&networkMessage, 0, sizeof(UA_NetworkMessage));
    size_t currentPosition = 0;
    UA_NetworkMessage_decodeBinary(&buffer, &currentPosition, &networkMessage);
    UA_ByteString_deleteMembers(&buffer);

    /* Is this the correct message type? */
    if(networkMessage.networkMessageType != UA_NETWORKMESSAGE_DATASET)
        goto cleanup;

    /* At least one DataSetMessage in the NetworkMessage? */
    if(networkMessage.payloadHeaderEnabled &&
       networkMessage.payloadHeader.dataSetPayloadHeader.count < 1)
        goto cleanup;

    /* Is this a KeyFrame-DataSetMessage? */
    UA_DataSetMessage *dsm = &networkMessage.payload.dataSetPayload.dataSetMessages[0];
    if(dsm->header.dataSetMessageType != UA_DATASETMESSAGE_DATAKEYFRAME)
        goto cleanup;

    /* Loop over the fields and print well-known content types */
    for(int i = 0; i < dsm->data.keyFrameData.fieldCount; i++) {
        const UA_DataType *currentType = dsm->data.keyFrameData.dataSetFields[i].value.type;
        if (currentType == &UA_TYPES[UA_TYPES_BYTESTRING]) {
            UA_ByteString *value = (UA_ByteString *)dsm->data.keyFrameData.dataSetFields[i].value.data;

            handleByteString(server, value);
        }
    }

    cleanup:
    UA_NetworkMessage_deleteMembers(&networkMessage);
}


static void
addSubscriber(UA_Server *server, UA_NodeId connectionIdent) {
    /* The following lines register the listening on the configured multicast
     * address and configure a repeated job, which is used to handle received
     * messages. */
    UA_PubSubConnection *connection =
            UA_PubSubConnection_findConnectionbyId(server, connectionIdent);
    if(connection != NULL) {
        UA_StatusCode rv = connection->channel->regist(connection->channel, NULL);
        /*if (rv == UA_STATUSCODE_GOOD) {
            UA_UInt64 subscriptionCallbackId;
            UA_Server_addRepeatedCallback(server, (UA_ServerCallback)subscriptionPollingCallback,
                                          connection, 5, &subscriptionCallbackId);
        } else {
            UA_LOG_WARNING(UA_Log_Stdout, UA_LOGCATEGORY_SERVER, "register channel failed: %s!",
                           UA_StatusCode_name(rv));
        }*/
    }

}

static void
addDataVariable(UA_Server *server, UA_ByteString *data) {
    UA_VariableAttributes attr = UA_VariableAttributes_default;
    attr.displayName = UA_LOCALIZEDTEXT("", "Binary Data");
    attr.accessLevel = UA_ACCESSLEVELMASK_READ | UA_ACCESSLEVELMASK_WRITE;

    UA_QualifiedName currentName = UA_QUALIFIEDNAME(1, "Binary Data");
    UA_NodeId parentNodeId = UA_NODEID_NUMERIC(0, UA_NS0ID_OBJECTSFOLDER);
    UA_NodeId parentReferenceNodeId = UA_NODEID_NUMERIC(0, UA_NS0ID_ORGANIZES);
    UA_NodeId variableTypeNodeId = UA_NODEID_NUMERIC(0, UA_NS0ID_BASEDATAVARIABLETYPE);

    UA_Variant_setScalar(&attr.value, data, &UA_TYPES[UA_TYPES_BYTESTRING]);
    attr.dataType = UA_TYPES[UA_TYPES_BYTESTRING].typeId;

    UA_StatusCode retVal = UA_Server_addVariableNode(server, payloadVariableNodeId, parentNodeId,
                                                     parentReferenceNodeId, currentName,
                                                     variableTypeNodeId, attr, NULL, NULL);
    if (retVal != UA_STATUSCODE_GOOD) {
        UA_LOG_WARNING(UA_Log_Stdout, UA_LOGCATEGORY_SERVER, "Adding node failed: %s!",
                       UA_StatusCode_name(retVal));
    }
}


static void
addPubSubConnection(UA_Server *server, UA_String *transportProfile,
                    UA_NetworkAddressUrlDataType *networkAddressUrl, UA_NodeId *connectionIdent){
    /* Details about the connection configuration and handling are located
     * in the pubsub connection tutorial */
    UA_PubSubConnectionConfig connectionConfig;
    memset(&connectionConfig, 0, sizeof(connectionConfig));
    connectionConfig.name = UA_STRING("UADP");
    connectionConfig.transportProfileUri = *transportProfile;
    connectionConfig.enabled = UA_TRUE;
    UA_Variant_setScalar(&connectionConfig.address, networkAddressUrl,
                         &UA_TYPES[UA_TYPES_NETWORKADDRESSURLDATATYPE]);
    connectionConfig.publisherId.numeric = UA_UInt32_random();
    UA_Server_addPubSubConnection(server, &connectionConfig, connectionIdent);
}

/**
 * **PublishedDataSet handling**
 *
 * The PublishedDataSet (PDS) and PubSubConnection are the toplevel entities and
 * can exist alone. The PDS contains the collection of the published fields. All
 * other PubSub elements are directly or indirectly linked with the PDS or
 * connection. */
static void
addPublishedDataSet(UA_Server *server, UA_NodeId *publishedDataSetIdent) {
    /* The PublishedDataSetConfig contains all necessary public
    * informations for the creation of a new PublishedDataSet */
    UA_PublishedDataSetConfig publishedDataSetConfig;
    memset(&publishedDataSetConfig, 0, sizeof(UA_PublishedDataSetConfig));
    publishedDataSetConfig.publishedDataSetType = UA_PUBSUB_DATASET_PUBLISHEDITEMS;
    publishedDataSetConfig.name = UA_STRING("PDS");
    /* Create new PublishedDataSet based on the PublishedDataSetConfig. */
    UA_Server_addPublishedDataSet(server, &publishedDataSetConfig, publishedDataSetIdent);
}

/**
 * **DataSetField handling**
 *
 * The DataSetField (DSF) is part of the PDS and describes exactly one published
 * field. */
static void
addDataSetField(UA_Server *server, UA_NodeId publishedDataSetIdent) {
    /* Add a field to the previous created PublishedDataSet */
    UA_NodeId dataSetFieldIdent;
    UA_DataSetFieldConfig dataSetFieldConfig;
    memset(&dataSetFieldConfig, 0, sizeof(UA_DataSetFieldConfig));
    dataSetFieldConfig.dataSetFieldType = UA_PUBSUB_DATASETFIELD_VARIABLE;
    dataSetFieldConfig.field.variable.fieldNameAlias = UA_STRING("");
    dataSetFieldConfig.field.variable.promotedField = UA_FALSE;
    dataSetFieldConfig.field.variable.publishParameters.publishedVariable = payloadVariableNodeId;
    dataSetFieldConfig.field.variable.publishParameters.attributeId = UA_ATTRIBUTEID_VALUE;
    UA_Server_addDataSetField(server, publishedDataSetIdent,
                              &dataSetFieldConfig, &dataSetFieldIdent);
}

/**
 * **WriterGroup handling**
 *
 * The WriterGroup (WG) is part of the connection and contains the primary
 * configuration parameters for the message creation. */
static void
addWriterGroup(UA_Server *server, UA_NodeId connectionIdent, UA_NodeId *writerGroupIdent) {
    /* Now we create a new WriterGroupConfig and add the group to the existing
     * PubSubConnection. */
    UA_WriterGroupConfig writerGroupConfig;
    memset(&writerGroupConfig, 0, sizeof(UA_WriterGroupConfig));
    writerGroupConfig.name = UA_STRING("WG");
    writerGroupConfig.publishingInterval = 0;
    writerGroupConfig.enabled = UA_FALSE;
    writerGroupConfig.writerGroupId = 100;
    writerGroupConfig.encodingMimeType = UA_PUBSUB_ENCODING_UADP;
    /* The configuration flags for the messages are encapsulated inside the
     * message- and transport settings extension objects. These extension
     * objects are defined by the standard. e.g.
     * UadpWriterGroupMessageDataType */
    UA_Server_addWriterGroup(server, connectionIdent, &writerGroupConfig, writerGroupIdent);
}

/**
 * **DataSetWriter handling**
 *
 * A DataSetWriter (DSW) is the glue between the WG and the PDS. The DSW is
 * linked to exactly one PDS and contains additional informations for the
 * message generation. */
static void
addDataSetWriter(UA_Server *server, UA_NodeId writerGroupIdent, UA_NodeId publishedDataSetIdent) {
    /* We need now a DataSetWriter within the WriterGroup. This means we must
     * create a new DataSetWriterConfig and add call the addWriterGroup function. */
    UA_NodeId dataSetWriterIdent;
    UA_DataSetWriterConfig dataSetWriterConfig;
    memset(&dataSetWriterConfig, 0, sizeof(UA_DataSetWriterConfig));
    dataSetWriterConfig.name = UA_STRING("DSW");
    dataSetWriterConfig.dataSetWriterId = 62541;
    dataSetWriterConfig.keyFrameCount = 10;
    UA_Server_addDataSetWriter(server, writerGroupIdent, publishedDataSetIdent,
                               &dataSetWriterConfig, &dataSetWriterIdent);
}


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

    UA_ServerConfig *config = UA_ServerConfig_new_minimal(16667, NULL);
    /* Details about the connection configuration and handling are located in
     * the pubsub connection tutorial */
    config->pubsubTransportLayers =
            (UA_PubSubTransportLayer *) UA_calloc(2, sizeof(UA_PubSubTransportLayer));
    if(!config->pubsubTransportLayers) {
        UA_ServerConfig_delete(config);
        return -1;
    }
    config->pubsubTransportLayers[0] = UA_PubSubTransportLayerUDPMP();
    config->pubsubTransportLayersSize++;
#ifdef UA_ENABLE_PUBSUB_ETH_UADP
    config->pubsubTransportLayers[1] = UA_PubSubTransportLayerEthernet();
    config->pubsubTransportLayersSize++;
#endif
    UA_Server *server = UA_Server_new(config);

    UA_Byte dataArr[PAYLOAD_SIZE];
    memset(dataArr, '*', PAYLOAD_SIZE);

    dataArr[0] = 1;
    UA_ByteString publishPayloadData;

    // init. the struct that will be sent to the server, and the variable that will hold the elapsed time for each call
    publishPayloadData.data = &dataArr[0];
    publishPayloadData.length = PAYLOAD_SIZE;




    UA_String transportProfile =
            UA_STRING("http://opcfoundation.org/UA-Profile/Transport/pubsub-udp-uadp");

    addDataVariable(server, &publishPayloadData);

    char subscribeHost[255];
    snprintf(subscribeHost, 255, "opc.udp://224.0.0.22:%d", server_port);

    UA_NetworkAddressUrlDataType subNetworkAddressUrl =
            {UA_STRING_NULL , UA_STRING(subscribeHost)};
    addPubSubConnection(server, &transportProfile, &subNetworkAddressUrl, &subConnectionIdent);
    addSubscriber(server, subConnectionIdent);



    UA_NodeId writerGroupIdent[PARALLEL_FORWARD];
    UA_NodeId publishedDataSetIdent[PARALLEL_FORWARD];
    UA_NodeId pubConnectionIdent[PARALLEL_FORWARD];
    UA_NetworkAddressUrlDataType pubNetworkAddressUrl[PARALLEL_FORWARD];
    char publishUrls[PARALLEL_FORWARD][255];




    for (unsigned int i = 0; i < PARALLEL_FORWARD; i++) {


        snprintf(publishUrls[i], 255, "%s:%d", start_endpoint, start_port + i);

        pubNetworkAddressUrl[i].networkInterface = UA_STRING_NULL;
        pubNetworkAddressUrl[i].url = UA_STRING(publishUrls[i]);

        addPubSubConnection(server, &transportProfile, &(pubNetworkAddressUrl[i]), &(pubConnectionIdent[i]));
        addPublishedDataSet(server, &(publishedDataSetIdent[i]));
        addDataSetField(server, publishedDataSetIdent[i]);
        addWriterGroup(server, pubConnectionIdent[i], &(writerGroupIdent[i]));
        addDataSetWriter(server, writerGroupIdent[i], publishedDataSetIdent[i]);

    }


    UA_PubSubConnection *connection =
            UA_PubSubConnection_findConnectionbyId(server, subConnectionIdent);

    UA_Server_run_startup(server);
    UA_Server_run_iterate(server, false);
    uint64_t elapsedTime[RUNS];

    for (unsigned int k = 0; k < RUNS && running; k++) {

        bool success = false;

        unsigned int timeoutMicros = 500 * 1000;
        uint64_t start_time;
        do {
            dataReceived = 0;
            UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_USERLAND, "[%d/%d] Publishing data", k, RUNS);
            start_time = get_microseconds();
            for (unsigned int i = 0; i < PARALLEL_FORWARD; i++) {
                manuallyPublish(server, writerGroupIdent[i]);
            }

            while (running && !success && (get_microseconds() - start_time) < timeoutMicros) {
                subscriptionPollingCallback(server, connection);
                success = dataReceived == PARALLEL_FORWARD * PAYLOAD_SIZE;
                if (!success)
                    usleep(10);
            }
            if (!success) {
                printf("TIMEOUT! Only got %d of %d bytes\n", dataReceived, PARALLEL_FORWARD * PAYLOAD_SIZE);
            }
        } while (running && !success);
        elapsedTime[k] = get_microseconds() - start_time;
        UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_USERLAND, "[%d/%d] All data received.", k, RUNS);
    }

    if (running) {
        uint64_t totalTime = 0;
        printf("-------------\nNode;Microseconds\n");
        for (unsigned int k = 0; k < RUNS; k++) {
            printf("%d;%ld\n", k, elapsedTime[k]);
            if (k > 0) // skip first, since it also includes setting up connection
                totalTime += elapsedTime[k];
        }
        printf("---------------\n");
        UA_LOG_WARNING(UA_Log_Stdout, UA_LOGCATEGORY_USERLAND, "Total Elapsed = %fms, Average = %fms",
                       totalTime / 1000.0, (totalTime / (double) (RUNS - 1)) / 1000.0);
    }

    running = false;
    UA_Server_run_shutdown(server);
    // bye bye
    UA_Server_delete(server);
    UA_ServerConfig_delete(config);
    return (int) UA_STATUSCODE_GOOD;
}
