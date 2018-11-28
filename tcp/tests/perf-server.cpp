
#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <errno.h>
#include <time.h>
#include <netinet/tcp.h>

#include "proc_stat.h"

bool running = true;
static void stopHandler(int sig) {
    running = false;
}


int main(int argc, char *argv[]) {


    signal(SIGINT, stopHandler);
    signal(SIGTERM, stopHandler);

    setProcPriority();



    char procStatFile[255];
    struct tm *timenow;
    time_t now = time(NULL);
    timenow = gmtime(&now);
    strftime(procStatFile, 255, "procStat-tcp-%Y-%m-%d_%H%M%S.csv",timenow);


    int procStatPid = runProcStatToCsv(procStatFile);
    if (procStatPid <= 0)
        return 1;



    // port to start the server on
    int SERVER_PORT = 8877;

    // socket address used for the server
    struct sockaddr_in server_address;
    memset(&server_address, 0, sizeof(server_address));
    server_address.sin_family = AF_INET;

    // htons: host to network short: transforms a value in host byte
    // ordering format to a short value in network byte ordering format
    server_address.sin_port = htons(SERVER_PORT);

    // htonl: host to network long: same as htons but to long
    server_address.sin_addr.s_addr = htonl(INADDR_ANY);

    // create a TCP socket, creation returns -1 on failure
    int listen_sock;
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

    // socket address used to store client address
    struct sockaddr_in client_address;
    socklen_t client_address_len = 0;

    printf("Ready to echo data\n");

    // open a new socket to transmit data per connection
    int sock;
    if ((sock =
                 accept(listen_sock, (struct sockaddr *)&client_address,
                        &client_address_len)) < 0) {
        printf("could not open a socket to accept data: %s\n", strerror(errno));
        return 1;
    }

    int i = 1;
    setsockopt( sock, IPPROTO_TCP, TCP_NODELAY, (void *)&i, sizeof(i));

    int n = 0;
    int maxlen = 500000;
    char buffer[maxlen];

    printf("client connected with ip address: %s\n",
           inet_ntoa(client_address.sin_addr));

    unsigned char ack = 1;


    // run indefinitely
    while (running) {

        bool isFirst = true;
        bool isEcho = true;
        // keep running as long as the client keeps the connection open
        while ((n = recv(sock, buffer, maxlen, 0)) > 0) {

            //int i = 1;
            //setsockopt( sock, IPPROTO_TCP, TCP_QUICKACK, (void *)&i, sizeof(i));
            if (n <= 0)
                continue;
            //printf("Received %d bytes", n);

            if (isFirst) {
                isEcho = buffer[0] == 0;
                isFirst = false;
            }

            if (isEcho) {
                // echo received content back
                int sentSize = send(sock, buffer, n, 0);
                if (sentSize != n) {
                    printf("sending echo failed: %s\n", strerror(errno));
                }
            }
            else {
                if (send(sock, &ack, 1, 0) != 1) {
                    printf("sending echo failed: %s\n", strerror(errno));
                }
            }
        }

    }

    close(sock);
    close(listen_sock);



    stopProcToCsv(procStatPid);

    printf("Process Status written to: %s\n", procStatFile);
    return 0;
}