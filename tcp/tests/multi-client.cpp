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


static void * recloop(void * data) {

    int *sockPtr = (int*) data;
    int sock = *sockPtr;

    ssize_t n = 0;
    int maxlen = 500000;
    char buffer[maxlen];
    while (running) {
        // keep running as long as the client keeps the connection open
        while ((n = recv(sock, buffer, maxlen, 0)) > 0) {
            //int i = 1;
            //setsockopt( sock, IPPROTO_TCP, TCP_QUICKACK, (void *)&i, sizeof(i));
            //printf("Received data %ld\n", n);
            pthread_mutex_lock(&mutex);
            dataReceived += n;
            pthread_mutex_unlock(&mutex);
        }
        if (n < 0) {
            printf("could not recv: %s\n", strerror(errno));

        }

    }
    close(sock);
}

static void * serverloop(void * _) {


    // socket address used to store client address
    struct sockaddr_in client_address;
    socklen_t client_address_len = 0;

    // open a new socket to transmit data per connection

    std::vector<pthread_t> recThreads;
    std::vector<int*> socks;
    while (running) {
        int sock;
        if ((sock =
                     accept(listen_sock, (struct sockaddr *) &client_address,
                            &client_address_len)) < 0) {
            printf("could not open a socket to accept data: %s\n", strerror(errno));
            return NULL;
        }


        printf("client connected with ip address: %s\n",
               inet_ntoa(client_address.sin_addr));

        pthread_t recThread;

        int *sockPtr = (int*)malloc(sizeof(int));
        *sockPtr = sock;
        socks.push_back(sockPtr);

        pthread_create(&recThread, NULL, &recloop, sockPtr);

        recThreads.push_back(recThread);
    }

    for(std::vector<pthread_t>::iterator it = recThreads.begin(); it != recThreads.end(); ++it) {
        pthread_join(*it, NULL);
    }

    for(std::vector<int*>::iterator it = socks.begin(); it != socks.end(); ++it) {
        free(*it);
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

static void *clientLoop(void *data) {

    int *sockPtr = (int *) data;


    //printf("Sending to socket %d\n", *sockPtr);

    ssize_t sent = send(*sockPtr, dataArr, PAYLOAD_SIZE, 0);
    if (sent != PAYLOAD_SIZE) {
        printf("Sent size does not match expected size: %s\n", strerror(errno));
        return NULL;
    }

    //printf("Sent %d\n", sent);

    return NULL;
}


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
    if ((listen_sock = socket(PF_INET, SOCK_STREAM, 0)) < 0) {
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

    int wait_size = 16;  // maximum number of waiting clients, after which
    // dropping begins
    if (listen(listen_sock, wait_size) < 0) {
        printf("could not open socket for listening: %s\n", strerror(errno));
        return 1;
    }


    pthread_t clientThreads[PARALLEL_FORWARD];
    unsigned int clientSock[PARALLEL_FORWARD];


    pthread_create(&server_thread, NULL, &serverloop, NULL);

    for (unsigned int i = 0; i < PARALLEL_FORWARD; i++) {


        struct sockaddr_in server_address;
        memset(&server_address, 0, sizeof(server_address));
        server_address.sin_family = AF_INET;

        // creates binary representation of server name
        // and stores it as sin_addr
        // http://beej.us/guide/bgnet/output/html/multipage/inet_ntopman.html
        inet_pton(AF_INET, forward_host, &server_address.sin_addr);

        // htons: port in network order format
        server_address.sin_port = htons(start_port + i);

        // open a stream socket
        if ((clientSock[i] = socket(PF_INET, SOCK_STREAM, 0)) < 0) {
            printf("could not create socket: %s\n", strerror(errno));
            return 1;
        }

        // TCP is connection oriented, a reliable connection
        // **must** be established before any data is exchanged
        if (connect(clientSock[i], (struct sockaddr*)&server_address,
                    sizeof(server_address)) < 0) {
            printf("could not connect to server %s:%d : %s\n", forward_host, start_port + i,  strerror(errno));
            return 1;
        }

        int opt = 1;
        setsockopt( clientSock[i], IPPROTO_TCP, TCP_NODELAY, (void *)&opt, sizeof(opt));
    }

    // Allow all the connections to connect
    sleep(5);

    uint64_t elapsedTime[RUNS];

    for (unsigned int k = 0; k < RUNS; k++) {
        dataReceived = 0;

        printf("[%d/%d] Publishing data\n", k, RUNS);
        uint64_t start_time = get_microseconds();
        for (unsigned int i = 0; i < PARALLEL_FORWARD; i++) {
            ssize_t sent = send(clientSock[i], dataArr, PAYLOAD_SIZE, 0);
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
