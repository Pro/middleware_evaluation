//
// Created by ayhun on 01/02/18.
//
/* This work is licensed under a Creative Commons CCZero 1.0 Universal License.
 * See http://creativecommons.org/publicdomain/zero/1.0/ for more information. */
#include <stdio.h>
#include <math.h>
#include <time.h>
#include "open62541.h"

#include "test_wrapper.h"

static bool
perform_method_call(void *context, void *payloadData, size_t payloadSize, long long int *elapsedTime, bool isEcho){
    UA_Client *client = (UA_Client*)context;
    UA_ByteString *payload = (UA_ByteString*)payloadData;
    payload->length = payloadSize;


    /* Call a remote method */
    __time_t before, after;
    UA_StatusCode retval;
    UA_Variant input;
    UA_Variant_init(&input);
    UA_Variant_setScalar(&input, payload, &UA_TYPES[UA_TYPES_BYTESTRING]);
    input.storageType = UA_VARIANT_DATA_NODELETE;
    size_t outputSize;
    UA_Variant *output;

    struct timespec time_start;
    TIME_MEASURE_START(time_start);
    if (isEcho){
        retval = UA_Client_call(client, UA_NODEID_NUMERIC(0, UA_NS0ID_OBJECTSFOLDER),
                                        UA_NODEID_NUMERIC(1, 62541), 1, &input, &outputSize, &output);
    } else {
        retval = UA_Client_call(client, UA_NODEID_NUMERIC(0, UA_NS0ID_OBJECTSFOLDER),
                                        UA_NODEID_NUMERIC(1, 62542), 1, &input, &outputSize, &output);
    }
    TIME_MEASURE_DIFF_USEC(time_start, *elapsedTime);
    if(retval == UA_STATUSCODE_GOOD) {
        //printf("Method call was successfull, and %lu returned values available.\n", (unsigned long)outputSize);
        //printf("returned value is of length: %d\n", ((int)((UA_String *) output->data)->length) - 6);
        UA_Array_delete(output, outputSize, &UA_TYPES[UA_TYPES_VARIANT]);
    } else {
        printf("Method call was unsuccessfull: %s\n", UA_StatusCode_name(retval));
    }
    UA_Variant_deleteMembers(&input);
    return retval == UA_STATUSCODE_GOOD;
}

int main(int argc, char *argv[]) {
    // intialize the large string that will be used for the function calls and set every char to '*'
    UA_Byte mainString[(int)pow(2,max_message_size_pow)];
    for(int i = 0; i < sizeof(mainString); i++){mainString[i] = '*';}
    // init. the struct that will be sent to the server, and the variable that will hold the elapsed time for each call
    UA_String argString = {.data = &mainString[0], .length = 1};
    long long int elapsed_time;
    // create a new client
    UA_Client *client = UA_Client_new(UA_ClientConfig_default);
    // Connect to a server
    UA_StatusCode retval = UA_Client_connect(client, "opc.tcp://cobot-t1-main:4840");
    if(retval != UA_STATUSCODE_GOOD) {
        UA_Client_delete(client);
        return (int)retval;
    }
    /* Init general purpose variables that will be used by all tests */

    /* For printing progress in a single line with dots */
    setbuf(stdout, NULL);

    bool success = runTests("opcua", &perform_method_call, client, &argString);

    // bye bye
    UA_Client_disconnect(client);
    UA_Client_delete(client);
    return (int) UA_STATUSCODE_GOOD;
}
