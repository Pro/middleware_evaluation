#include <arpa/inet.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include "test_wrapper.h"

int sock;
struct sockaddr_in server_address;

static bool
send_data(void *context, void *payloadData, size_t payloadSize, long long int *elapsedTime, bool isEcho) {
    // data that will be sent to the server
    char *data = (char *) payloadData;
    if (isEcho)
        data[0] = 0;
    else
        data[0] = 1;


    bool success = false;
    do {


        long count = 0;
        long countTimeout = 1000;

        struct timespec time_start;
        TIME_MEASURE_START(time_start);

        ssize_t sent = sendto(sock, (const char *) payloadData, payloadSize,
                              MSG_CONFIRM, (const struct sockaddr *) &server_address,
                              sizeof(server_address));
        if (sent != payloadSize) {
            printf("Sent size does not match expected size: %s\n", strerror(errno));
            return false;
        }

        // receive

        ssize_t n = 0;
        socklen_t len = sizeof(server_address);
        size_t received = 0, maxlen = 500000;
        char buffer[maxlen];

        int expectedSize = isEcho ? payloadSize : 1;


        //printf("Sent %ld bytes", sent);

        while (!success && count < countTimeout) {
            // will remain open until the server terminates the connection
            n = recvfrom(sock, (char *) buffer, maxlen,
                         MSG_WAITALL, (struct sockaddr *) &server_address,
                         &len);
            if (n > 0) {
                received += n;
                //printf("Receiving %ld bytes\n", n);
            }

            success = received >= expectedSize;
            count++;
        }


        //printf("Received %ld bytes", received);
        TIME_MEASURE_DIFF_USEC(time_start, *elapsedTime);
        if (!success)
            printf("Timeout. Resending ...");
    } while (!success);

    return true;
}

int main() {
    const char *server_name = "10.200.2.82";
    const int server_port = 8877;


    memset(&server_address, 0, sizeof(server_address));
    server_address.sin_family = AF_INET;

    char mainString[(int) pow(2, max_message_size_pow)];
    memset(mainString, '*', sizeof(mainString));

    //server_address.sin_addr.s_addr = INADDR_ANY;
    // creates binary representation of server name
    // and stores it as sin_addr
    // http://beej.us/guide/bgnet/output/html/multipage/inet_ntopman.html
    inet_pton(AF_INET, server_name, &server_address.sin_addr);

    // htons: port in network order format
    server_address.sin_port = htons(server_port);

    // open a stream socket
    if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        printf("could not create socket: %s\n", strerror(errno));
        return 1;
    }


    /* For printing progress in a single line with dots */
    setbuf(stdout, NULL);


    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 10;
    if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
        perror("Error");
    }


    bool success = runTests("udp", &send_data, NULL, mainString);

    // close the socket
    close(sock);
    return success == true;
}
