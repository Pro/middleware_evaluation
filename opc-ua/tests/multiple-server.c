#include <signal.h>
#include <stdio.h>
#include <string.h>
#include "open62541.h"
#include <pthread.h>

UA_Boolean running = true;

static void stopHandler(int sig) {
    UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_USERLAND, "received ctrl-c");
    running = false;
}

pthread_t server_thread;
UA_Server *server;

static void * serverloop(void * _) {
        while(running)
            UA_Server_run_iterate(server, true);
        return 0;
}

UA_Client *forwardWriteClient = NULL;
char *forwardEndpoint;
UA_UInt16 selfPort;


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

    // now write payload to the next node

    for (unsigned int i=0; i<2; i++) {
        if (UA_Client_getState(forwardWriteClient) != UA_CLIENTSTATE_CONNECTED) {

            /* Connect to a server */
            /* anonymous connect would be: retval = UA_Client_connect(client, "opc.tcp://localhost:4840"); */
            UA_StatusCode retval = UA_Client_connect(forwardWriteClient, forwardEndpoint);
            if (retval != UA_STATUSCODE_GOOD) {
                UA_Client_delete(forwardWriteClient);
                UA_LOG_ERROR(UA_Log_Stdout, UA_LOGCATEGORY_USERLAND, "Cannot connect to %s", forwardEndpoint);
                return (int) retval;
            }

        }

        /*UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_USERLAND,
                     "[%d] Send Data to %s",selfPort,  forwardEndpoint);*/

        UA_StatusCode retval = UA_Client_writeValueAttribute(forwardWriteClient, UA_NODEID_NUMERIC(1, 1000),
                                                             &data->value);

        if(retval == UA_STATUSCODE_BADCONNECTIONCLOSED) {
            UA_LOG_ERROR(UA_Log_Stdout, UA_LOGCATEGORY_CLIENT, "Connection was closed. Reconnecting ...");
            continue;
        }
        if (retval != UA_STATUSCODE_GOOD)
            return retval;
        break;
    }



    return UA_STATUSCODE_GOOD;
}

static UA_StatusCode
readDataFromForward(UA_Server *server, const UA_NodeId *sessionId,
                    void *sessionContext, const UA_NodeId *nodeId,
                    void *nodeContext, UA_Boolean includeSourceTimeStamp,
                    const UA_NumericRange *range, UA_DataValue *value) {

    UA_ByteString bytes = UA_BYTESTRING_NULL;
    UA_Variant_setScalar(&value->value, &bytes, &UA_TYPES[UA_TYPES_BYTESTRING]);
    return UA_STATUSCODE_GOOD;
}


static void
addForwardNode(UA_Server *server) {
    UA_VariableAttributes attr = UA_VariableAttributes_default;
    attr.displayName = UA_LOCALIZEDTEXT("en-US", "ForwardData");
    attr.accessLevel = UA_ACCESSLEVELMASK_READ | UA_ACCESSLEVELMASK_WRITE;

    UA_ByteString bytes = UA_BYTESTRING_NULL;
    UA_Variant_setScalar(&attr.value, &bytes, &UA_TYPES[UA_TYPES_BYTESTRING]);



    UA_DataSource forwardDataSource;
    forwardDataSource.read = readDataFromForward;
    forwardDataSource.write = writeDataToForward;
    UA_StatusCode retval = UA_Server_addDataSourceVariableNode(server,
                                                               UA_NODEID_NUMERIC(1, 1000),
                                                               UA_NODEID_NUMERIC(0, UA_NS0ID_OBJECTSFOLDER),
                                                               UA_NODEID_NUMERIC(0, UA_NS0ID_ORGANIZES),
                                                               UA_QUALIFIEDNAME(1, "ForwardData"),
                                                               UA_NODEID_NUMERIC(0, UA_NS0ID_BASEDATAVARIABLETYPE), attr,
                                                               forwardDataSource, NULL, NULL);

    if (retval != UA_STATUSCODE_GOOD) {
        UA_LOG_ERROR(UA_Log_Stdout, UA_LOGCATEGORY_SERVER, "Cannot create variable node. %s", UA_StatusCode_name(retval));
    }
}


// -----------------------------------------------------------------------------------------
int main(int argc, char *argv[]) {
    signal(SIGINT, stopHandler);
    signal(SIGTERM, stopHandler);

    if (argc != 3) {
        UA_LOG_ERROR(UA_Log_Stdout, UA_LOGCATEGORY_USERLAND, "Usage: program PORT FORWARD_ENDPOINT");
        return 1;
    }

    selfPort = atoi(argv[1]);
    if (selfPort == 0) {
        UA_LOG_ERROR(UA_Log_Stdout, UA_LOGCATEGORY_USERLAND, "Invalid Port");
        return 1;
    }

    forwardEndpoint = argv[2];

    UA_ServerConfig *config = UA_ServerConfig_new_minimal(selfPort, NULL);
    server = UA_Server_new(config);

    addForwardNode(server);

    forwardWriteClient = UA_Client_new(UA_ClientConfig_default);

    UA_Server_run_startup(server);

    pthread_create(&server_thread, NULL, &serverloop, NULL);

    pthread_join(server_thread, NULL);

    UA_Client_delete(forwardWriteClient);
    UA_Server_delete(server);
    UA_ServerConfig_delete(config);
    return (int) 0;
}
