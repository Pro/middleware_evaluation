
#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <errno.h>
#include <time.h>

#include "proc_stat.h"
#include <unistd.h>
#include <fcntl.h>

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
    strftime(procStatFile, 255, "procStat-udp-%Y-%m-%d_%H%M%S.csv", timenow);


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
    if ((listen_sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        printf("could not create listen socket: %s\n", strerror(errno));
        return 1;
    }

    int opt = 1;

    // Forcefully attaching socket to the port 8080
    if (setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT,
                   &opt, sizeof(opt))) {
        perror("setsockopt");
        exit(EXIT_FAILURE);
    }

    // bind it to listen to the incoming connections on the created server
    // address, will return -1 on error
    if ((bind(listen_sock, (struct sockaddr *) &server_address,
              sizeof(server_address))) < 0) {
        printf("could not bind socket: %s\n", strerror(errno));
        return 1;
    }

    // socket address used to store client address
    struct sockaddr_in client_address;
    socklen_t client_address_len = sizeof(client_address);


    printf("Ready to echo data\n");

    ssize_t n = 0;
    int maxlen = 500000;
    char buffer[maxlen];
    unsigned char ack = 1;

    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000;
    if (setsockopt(listen_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
        perror("Error");
    }

    // run indefinitely
    while (running) {

        bool isFirst = true;
        bool isEcho = true;

        // keep running as long as the client keeps the connection open
        if ((n = recvfrom(listen_sock, (char *) buffer, maxlen,
                          MSG_WAITALL, (struct sockaddr *) &client_address,
                          &client_address_len)) > 0) {


            //printf("client connected with ip address %s and sent %ld bytes\n",
            //       inet_ntoa(client_address.sin_addr), n);


            if (n <= 0)
                continue;
            //printf("Received %d bytes", n);

            if (isFirst) {
                isEcho = buffer[0] == 0;
                isFirst = false;
            }

            if (isEcho) {
                // echo received content back
                //printf("sending %ld echo bytes\n", n);

                ssize_t sentSize = sendto(listen_sock, (const char *) buffer, n,
                                          MSG_CONFIRM, (const struct sockaddr *) &client_address,
                                          client_address_len);
                if (sentSize != n) {
                    printf("sending echo failed: %s\n", strerror(errno));
                }
            } else {
                //printf("sending 1 ack byte\n");

                ssize_t sentSize = sendto(listen_sock, (const char *) &ack, 1,
                                          MSG_CONFIRM, (const struct sockaddr *) &client_address,
                                          client_address_len);;

                if (sentSize != 1) {
                    printf("sending ack failed: %s\n", strerror(errno));
                }
            }
        }

    }

    close(listen_sock);

    stopProcToCsv(procStatPid);

    printf("Process Status written to: %s\n", procStatFile);
    return 0;
}