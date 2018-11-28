//
// Created by profanter on 9/12/18.
// Copyright (c) 2018 fortiss GmbH. All rights reserved.
//

#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <errno.h>
#include <time.h>
#include <time.h>
#include <math.h>
#include <unistd.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <netinet/tcp.h>

bool running = true;

static void stopHandler(int sig) {
    printf("received ctrl-c\n");
    running = false;
}



pthread_t server_thread;
int listen_sock;
int send_sock;


struct sockaddr_in remote_address;

static void * serverloop(void * _) {

    while(running) {
        //printf("Ready to echo data\n");

        // open a new socket to transmit data per connection


        ssize_t n = 0;
        int maxlen = 50000;
        char buffer[maxlen];
        struct sockaddr_in server_address;
        socklen_t len = sizeof(server_address);

        // keep running as long as the client keeps the connection open
        while(running) {
            while ((n = recvfrom(listen_sock, (char *) buffer, maxlen,
                                 MSG_WAITALL, (struct sockaddr *) &server_address,
                                 &len)) > 0) {

                //int i = 1;
                //setsockopt( sock, IPPROTO_TCP, TCP_QUICKACK, (void *)&i, sizeof(i));

                //printf("Received %d bytes\n", n);
                if (n == 0)
                    continue;

                //printf("Send back\n");
                // echo received content back

                ssize_t sentSize = sendto(send_sock, (const char *) buffer, n,
                                      MSG_CONFIRM, (const struct sockaddr *) &remote_address,
                                      sizeof(remote_address));
                if (sentSize != n) {
                    printf("sending echo failed: %s\n", strerror(errno));
                }
            }
            /*if (n < 0) {

                printf("rec client failed %s\n", strerror(errno));
                break;
            }*/
        }

    }

    return 0;
}

// -----------------------------------------------------------------------------------------
int main(int argc, char *argv[]) {
    signal(SIGINT, stopHandler);
    signal(SIGTERM, stopHandler);

    if (argc != 4) {
        printf("Usage: program PORT FORWARD_HOST FORWARD_PORT\n");
        return 1;
    }

    int selfPort = atoi(argv[1]);
    if (selfPort == 0) {
        printf("Invalid self Port\n");
        return 1;
    }

    char *forwardHost = argv[2];
    if (strlen(forwardHost) == 0) {

    }

    int forwardPort = atoi(argv[3]);
    if (forwardPort == 0) {
        printf("Invalid forward Port\n");
        return 1;
    }

    // socket address used for the server
    struct sockaddr_in server_address;
    memset(&server_address, 0, sizeof(server_address));
    server_address.sin_family = AF_INET;

    // htons: host to network short: transforms a value in host byte
    // ordering format to a short value in network byte ordering format
    server_address.sin_port = htons(selfPort);

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




    // open a stream socket
    if ((send_sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        printf("could not create socket: %s\n", strerror(errno));
        return 1;
    }


    memset(&remote_address, 0, sizeof(remote_address));
    remote_address.sin_family = AF_INET;
    // creates binary representation of server name
    // and stores it as sin_addr
    // http://beej.us/guide/bgnet/output/html/multipage/inet_ntopman.html
    inet_pton(AF_INET, forwardHost, &remote_address.sin_addr);

    // htons: port in network order format
    remote_address.sin_port = htons(forwardPort);

    pthread_create(&server_thread, NULL, &serverloop, NULL);

    pthread_join(server_thread, NULL);

    close(listen_sock);
    close(send_sock);


    return (int) 0;
}
