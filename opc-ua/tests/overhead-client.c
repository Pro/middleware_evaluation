//
// Created by profanter on 8/23/18.
// Copyright (c) 2018 fortiss GmbH. All rights reserved.
//

#include <stdio.h>
#include <math.h>
#include <time.h>
#include "open62541.h"

int main(int argc, char *argv[]) {

    if (argc != 3) {
        printf("Usage: program <ENDPOINT_URL> <PAYLOAD_BYTES>\n");
        return 1;
    }

    int payloadSize = atoi(argv[2]);

    if (payloadSize < 0){
        printf("Payload size cannot be negative\n");
        return 1;
    }

    printf("Peparing payload of %d bytes to %s\n", payloadSize, argv[1]);

    UA_Client *client = UA_Client_new(UA_ClientConfig_default);
    UA_StatusCode retval = UA_Client_connect(client, argv[1]);
    if(retval != UA_STATUSCODE_GOOD) {
        UA_Client_delete(client);
        return (int)retval;
    }

    UA_ByteString data;

    if (UA_ByteString_allocBuffer(&data, payloadSize) != UA_STATUSCODE_GOOD) {
        printf("Cannot alloc byte buffer\n");
        return 1;
    }

    srand((unsigned int) time(NULL));
    for (int i=0; i<payloadSize; i++)
        data.data[i] = (UA_Byte)(rand()%255);


    UA_Variant *myVariant = UA_Variant_new();
    UA_Variant_setScalarCopy(myVariant, &data, &UA_TYPES[UA_TYPES_BYTESTRING]);

    printf("Sending payload of %d\n", payloadSize);
    UA_StatusCode ret = UA_Client_writeValueAttribute(client, UA_NODEID_NUMERIC(1, 1000), myVariant);
    printf("Sent payload. Return value %s\n", UA_StatusCode_name(ret));
    UA_Variant_delete(myVariant);


    UA_Client_delete(client); /* Disconnects the client internally */
    return UA_STATUSCODE_GOOD;
}