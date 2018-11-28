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
bool clientConnected = false;

static void * serverloop(void * _) {

    // socket address used to store client address
    struct sockaddr_in client_address;
    socklen_t client_address_len = 0;

    while(running) {
        //printf("Ready to echo data\n");

        // open a new socket to transmit data per connection
        int sock;
        if ((sock =
                     accept(listen_sock, (struct sockaddr *) &client_address,
                            &client_address_len)) < 0) {
            printf("could not open a socket to accept data: %s\n", strerror(errno));
            return NULL;
        }


        int n = 0;
        int maxlen = 50000;
        char buffer[maxlen];

        printf("client connected with ip address: %s\n",
               inet_ntoa(client_address.sin_addr));


        while (!clientConnected) {
            sleep(1);
        }

        // keep running as long as the client keeps the connection open
        while(running) {
            while ((n = recv(sock, buffer, maxlen, 0)) > 0) {

                //int i = 1;
                //setsockopt( sock, IPPROTO_TCP, TCP_QUICKACK, (void *)&i, sizeof(i));

                //printf("Received %d bytes\n", n);
                if (n == 0)
                    continue;

                //printf("Send back\n");
                // echo received content back
                int sentSize = send(send_sock, buffer, n, 0);
                if (sentSize != n) {
                    printf("sending echo failed: %s\n", strerror(errno));
                }
            }
            if (n < 0) {

                printf("rec client failed %s\n", strerror(errno));
                break;
            }
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


    pthread_create(&server_thread, NULL, &serverloop, NULL);


    struct sockaddr_in remote_address;
    memset(&remote_address, 0, sizeof(remote_address));
    remote_address.sin_family = AF_INET;

    // creates binary representation of server name
    // and stores it as sin_addr
    // http://beej.us/guide/bgnet/output/html/multipage/inet_ntopman.html
    inet_pton(AF_INET, forwardHost, &remote_address.sin_addr);

    // htons: port in network order format
    remote_address.sin_port = htons(forwardPort);

    clientConnected = false;
    // open a stream socket
    if ((send_sock = socket(PF_INET, SOCK_STREAM, 0)) < 0) {
        printf("could not create socket: %s\n", strerror(errno));
        return 1;
    }

    // TCP is connection oriented, a reliable connection
    // **must** be established before any data is exchanged
    while (connect(send_sock, (struct sockaddr*)&remote_address,
                   sizeof(remote_address)) < 0) {
        if (errno == ECONNREFUSED) {
            sleep(1);
            continue;
        }
        printf("could not connect to server: %s\n", strerror(errno));
        return 1;
    }
    int i = 1;
    setsockopt( send_sock, IPPROTO_TCP, TCP_NODELAY, (void *)&i, sizeof(i));
    clientConnected = true;




    pthread_join(server_thread, NULL);

    close(listen_sock);
    close(send_sock);


    return (int) 0;
}
