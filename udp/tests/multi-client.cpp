//
// Created by profanter on 9/12/18.
// Copyright (c) 2018 fortiss GmbH. All rights reserved.
//


#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <errno.h>
#include <time.h>
#include <time.h>
#include <math.h>
#include <unistd.h>
#include <pthread.h>
#include <vector>
#include <netinet/tcp.h>

bool running = true;

static void stopHandler(int sig) {
    printf("received ctrl-c\n");
    running = false;
}

long dataReceived = 0;



int listen_sock;

pthread_t server_thread;

pthread_mutex_t mutex;


static void * serverloop(void * _) {

    // open a new socket to transmit data per connection

    while (running) {
        ssize_t n = 0;
        int maxlen = 500000;
        char buffer[maxlen];
        struct sockaddr_in server_address;
        socklen_t len = sizeof(server_address);
        while (running) {
            // keep running as long as the client keeps the connection open
            while ((n = recvfrom(listen_sock, (char *) buffer, maxlen,
                                 MSG_WAITALL, (struct sockaddr *) &server_address,
                                 &len)) > 0) {
                pthread_mutex_lock(&mutex);
                dataReceived += n;
                pthread_mutex_unlock(&mutex);
            }
            /*if (n < 0) {
                printf("could not recv: %s\n", strerror(errno));

            }*/
        }
    }

    return 0;
}

uint64_t get_microseconds() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
    return ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
}


#define PAYLOAD_SIZE 10240
#define PARALLEL_FORWARD 10
#define RUNS 100
char dataArr[PAYLOAD_SIZE];

char *forward_host;

// -----------------------------------------------------------------------------------------
int main(int argc, char *argv[]) {
    signal(SIGINT, stopHandler);
    signal(SIGTERM, stopHandler);

    if (argc != 4) {
        printf("Usage: program SERVER_PORT CLIENT_START_PORT FORWARD_HOST\n");
        return 1;
    }

    int server_port = atoi(argv[1]);
    if (server_port == 0) {
        printf("Invalid Server Port\n");
        return 1;
    }

    int start_port = atoi(argv[2]);
    if (start_port == 0) {
        printf("Invalid Client Port\n");
        return 1;
    }

    forward_host = argv[3];

    memset(dataArr, '*', PAYLOAD_SIZE);



    // socket address used for the server
    struct sockaddr_in server_address;
    memset(&server_address, 0, sizeof(server_address));
    server_address.sin_family = AF_INET;

    // htons: host to network short: transforms a value in host byte
    // ordering format to a short value in network byte ordering format
    server_address.sin_port = htons(server_port);

    // htonl: host to network long: same as htons but to long
    server_address.sin_addr.s_addr = htonl(INADDR_ANY);

    // create a TCP socket, creation returns -1 on failure
    if ((listen_sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        printf("could not create listen socket: %s\n", strerror(errno));
        return 1;
    }

    int opt = 1;

    // Forcefully attaching socket to the port 8080
    if (setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT,
                   &opt, sizeof(opt)))
    {
        perror("setsockopt");
        exit(EXIT_FAILURE);
    }

    // bind it to listen to the incoming connections on the created server
    // address, will return -1 on error
    if ((bind(listen_sock, (struct sockaddr *)&server_address,
              sizeof(server_address))) < 0) {
        printf("could not bind socket: %s\n", strerror(errno));
        return 1;
    }

    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 10;
    if (setsockopt(listen_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
        perror("Error");
    }

    unsigned int clientSock[PARALLEL_FORWARD];
    struct sockaddr_in clientAddress[PARALLEL_FORWARD];

    pthread_create(&server_thread, NULL, &serverloop, NULL);

    for (unsigned int i = 0; i < PARALLEL_FORWARD; i++) {
        memset(&clientAddress[i], 0, sizeof(clientAddress[i]));
        clientAddress[i].sin_family = AF_INET;

        // creates binary representation of server name
        // and stores it as sin_addr
        // http://beej.us/guide/bgnet/output/html/multipage/inet_ntopman.html
        inet_pton(AF_INET, forward_host, &clientAddress[i].sin_addr);

        // htons: port in network order format
        clientAddress[i].sin_port = htons(start_port + i);

        // open a stream socket
        if ((clientSock[i] = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
            printf("could not create socket: %s\n", strerror(errno));
            return 1;
        }
    }

    // Allow all the connections to connect
    sleep(5);

    uint64_t elapsedTime[RUNS];

    for (unsigned int k = 0; k < RUNS; k++) {
        dataReceived = 0;

        printf("[%d/%d] Publishing data\n", k, RUNS);
        uint64_t start_time = get_microseconds();
        for (unsigned int i = 0; i < PARALLEL_FORWARD; i++) {

            ssize_t sent = sendto(clientSock[i], (const char *) dataArr, PAYLOAD_SIZE,
                                  MSG_CONFIRM, (const struct sockaddr *) &clientAddress[i],
                                  sizeof(clientAddress[i]));
            if (sent != PAYLOAD_SIZE) {
                printf("Sent size does not match expected size: %s\n", strerror(errno));
                return 1;
            }
        }

        while (dataReceived != PARALLEL_FORWARD * PAYLOAD_SIZE) {
            usleep(10);
        }
        elapsedTime[k] = get_microseconds() - start_time;
        printf("[%d/%d] All data received.\n", k, RUNS);
    }

    for (unsigned int i = 0; i < PARALLEL_FORWARD; i++) {
        close(clientSock[i]);
    }


    uint64_t totalTime = 0;
    printf("-------------\nNode;Microseconds\n");
    for (unsigned int k = 0; k < RUNS; k++) {
        printf("%d;%ld\n",k, elapsedTime[k]);
        if (k > 0) // skip first, since it also includes setting up connection
            totalTime += elapsedTime[k];
    }
    printf("---------------\n");
    printf("Total Elapsed = %fms, Average = %fms\n", totalTime/1000.0, (totalTime / (double)(RUNS-1))/1000.0);



    running = false;

    pthread_join(server_thread, NULL);
    return (int) 0;
}
