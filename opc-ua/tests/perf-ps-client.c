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

UA_NodeId pubConnectionIdent, subConnectionIdent, publishedDataSetIdent, publishedDataSetIdent, writerGroupIdent;

#include "test_wrapper.h"

UA_ByteString publishPayloadData = {0, NULL};
UA_ByteString fullPayloadData = {0, NULL};;

UA_NodeId payloadVariableNodeId = {
        .namespaceIndex = 1,
        .identifierType = UA_NODEIDTYPE_NUMERIC,
        .identifier.numeric = 50123
};

UA_Boolean running = true;
static void stopHandler(int sign) {
    UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_SERVER, "received ctrl-c");
    running = false;
}

static void manuallyPublish(UA_Server *server) {
    UA_WriterGroup *wg = UA_WriterGroup_findWGbyId(server, writerGroupIdent);
    UA_WriterGroup_publishCallback(server, wg);
}

bool dataReceived = false;


static void handleByteString(UA_Server *server, UA_ByteString* bs) {
    dataReceived = true;
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


static bool
perform_publish_subscribe(void *context, void *payloadData, size_t payloadSize, long long int *elapsedTime, bool isEcho){

    UA_Server *server = (UA_Server*) context;

    publishPayloadData.length = payloadSize;

    if (payloadSize > 0) {
        // for simplicity we set the first byte to 0 if it is echo, to 1 if it is ack
        publishPayloadData.data[0] = (UA_Byte) (isEcho ? 1 : 0);
    }

    UA_StatusCode retval = UA_STATUSCODE_GOOD;

    struct timespec time_start;


    UA_PubSubConnection *connection =
            UA_PubSubConnection_findConnectionbyId(server, subConnectionIdent);

    dataReceived = false;
    TIME_MEASURE_START(time_start);
    manuallyPublish(server);
    //printf("Waiting for reponse\n");
    while(!dataReceived && running) {
        subscriptionPollingCallback(server, connection);
    }
    TIME_MEASURE_DIFF_USEC(time_start, *elapsedTime);

    if(retval == UA_STATUSCODE_GOOD) {

    } else {
        printf("Method call was unsuccessfull: %s\n", UA_StatusCode_name(retval));
    }
    return retval == UA_STATUSCODE_GOOD && running;
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

static UA_StatusCode
readCurrentBinaryData(UA_Server *server,
                      const UA_NodeId *sessionId, void *sessionContext,
                      const UA_NodeId *nodeId, void *nodeContext,
                      UA_Boolean sourceTimeStamp, const UA_NumericRange *range,
                      UA_DataValue *dataValue) {

    //dataValue->value.storageType = UA_VARIANT_DATA_NODELETE;

    UA_Variant_setScalarCopy(&dataValue->value, &publishPayloadData,
                             &UA_TYPES[UA_TYPES_BYTESTRING]);
    dataValue->value.storageType = UA_VARIANT_DATA_NODELETE;
    dataValue->hasValue = true;
    return UA_STATUSCODE_GOOD;
}

static void
addDataVariable(UA_Server *server) {
    UA_VariableAttributes attr = UA_VariableAttributes_default;
    attr.displayName = UA_LOCALIZEDTEXT("", "Binary Data");
    attr.accessLevel = UA_ACCESSLEVELMASK_READ | UA_ACCESSLEVELMASK_WRITE;

    UA_QualifiedName currentName = UA_QUALIFIEDNAME(1, "Binary Data");
    UA_NodeId parentNodeId = UA_NODEID_NUMERIC(0, UA_NS0ID_OBJECTSFOLDER);
    UA_NodeId parentReferenceNodeId = UA_NODEID_NUMERIC(0, UA_NS0ID_ORGANIZES);
    UA_NodeId variableTypeNodeId = UA_NODEID_NUMERIC(0, UA_NS0ID_BASEDATAVARIABLETYPE);

    UA_DataSource binaryDataSource;
    binaryDataSource.read = readCurrentBinaryData;
    binaryDataSource.write = NULL;
    UA_StatusCode retVal = UA_Server_addDataSourceVariableNode(server, payloadVariableNodeId, parentNodeId,
                                        parentReferenceNodeId, currentName,
                                        variableTypeNodeId, attr,
                                        binaryDataSource, NULL, NULL);

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
addPublishedDataSet(UA_Server *server) {
    /* The PublishedDataSetConfig contains all necessary public
    * informations for the creation of a new PublishedDataSet */
    UA_PublishedDataSetConfig publishedDataSetConfig;
    memset(&publishedDataSetConfig, 0, sizeof(UA_PublishedDataSetConfig));
    publishedDataSetConfig.publishedDataSetType = UA_PUBSUB_DATASET_PUBLISHEDITEMS;
    publishedDataSetConfig.name = UA_STRING("PDS");
    /* Create new PublishedDataSet based on the PublishedDataSetConfig. */
    UA_Server_addPublishedDataSet(server, &publishedDataSetConfig, &publishedDataSetIdent);
}

/**
 * **DataSetField handling**
 *
 * The DataSetField (DSF) is part of the PDS and describes exactly one published
 * field. */
static void
addDataSetField(UA_Server *server) {
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
addWriterGroup(UA_Server *server, UA_NodeId connectionIdent) {
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
    UA_Server_addWriterGroup(server, connectionIdent, &writerGroupConfig, &writerGroupIdent);
}

/**
 * **DataSetWriter handling**
 *
 * A DataSetWriter (DSW) is the glue between the WG and the PDS. The DSW is
 * linked to exactly one PDS and contains additional informations for the
 * message generation. */
static void
addDataSetWriter(UA_Server *server) {
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
    // intialize the large string that will be used for the function calls and set every char to '*'
    UA_Byte mainString[(int)pow(2,max_message_size_pow)];
    for(int i = 0; i < sizeof(mainString); i++){mainString[i] = '*';}
    // init. the struct that will be sent to the server, and the variable that will hold the elapsed time for each call
    UA_String argString = {.data = &mainString[0], .length = 1};
    fullPayloadData.data = &mainString[0];
    fullPayloadData.length = (int)pow(2,max_message_size_pow);
    publishPayloadData.data = fullPayloadData.data;


    UA_String transportProfile =
            UA_STRING("http://opcfoundation.org/UA-Profile/Transport/pubsub-udp-uadp");
    UA_NetworkAddressUrlDataType networkAddressUrl =
            {UA_STRING_NULL , UA_STRING("opc.udp://224.0.0.22:4840/")};

    signal(SIGINT, stopHandler);
    signal(SIGTERM, stopHandler);

    UA_StatusCode retval = UA_STATUSCODE_GOOD;
    UA_ServerConfig *config = UA_ServerConfig_new_minimal(16660, NULL);
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

    addDataVariable(server);

    UA_NetworkAddressUrlDataType pubNetworkAddressUrl =
            {UA_STRING_NULL , UA_STRING("opc.udp://224.0.0.22:4841/")};
    addPubSubConnection(server, &transportProfile, &pubNetworkAddressUrl, &pubConnectionIdent);
    addPublishedDataSet(server);
    addDataSetField(server);
    addWriterGroup(server, pubConnectionIdent);
    addDataSetWriter(server);
    UA_NetworkAddressUrlDataType subNetworkAddressUrl =
            {UA_STRING_NULL , UA_STRING("opc.udp://224.0.0.22:4840/")};
    addPubSubConnection(server, &transportProfile, &subNetworkAddressUrl, &subConnectionIdent);
    addSubscriber(server, subConnectionIdent);


    UA_Server_run_startup(server);

    UA_Server_run_iterate(server, false);


    /* For printing progress in a single line with dots */
    setbuf(stdout, NULL);

    bool success = runTests("opcua-ps", &perform_publish_subscribe, server, &argString);

    running = false;
    UA_Server_run_shutdown(server);
    // bye bye
    UA_Server_delete(server);
    UA_ServerConfig_delete(config);
    return (int) UA_STATUSCODE_GOOD;
}
