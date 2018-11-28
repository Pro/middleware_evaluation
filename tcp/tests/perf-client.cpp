#include <arpa/inet.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include <netinet/tcp.h>

#include "test_wrapper.h"

int sock;


static bool
send_data(void *context, void *payloadData, size_t payloadSize, long long int *elapsedTime, bool isEcho){
    // data that will be sent to the server
    char *data = (char*) payloadData;
    if (isEcho)
        data[0] = 0;
    else
        data[0] = 1;

    struct timespec time_start;
    TIME_MEASURE_START(time_start);
    ssize_t sent = send(sock, payloadData, payloadSize, 0);
    if (sent != payloadSize) {
        printf("Sent size does not match expected size: %s\n", strerror(errno));
        return false;
    }

    // receive

    int n = 0;
    int received = 0, maxlen = 500000;
    char buffer[maxlen];

    int expectedSize = isEcho ? payloadSize : 1;

    // will remain open until the server terminates the connection
    while ( received < expectedSize && (n = recv(sock, buffer, maxlen, 0)) > 0) {
        received += n;
    }
    TIME_MEASURE_DIFF_USEC(time_start, *elapsedTime);

    return true;
}

int main() {
    const char* server_name = "10.200.2.82";
    const int server_port = 8877;

	printf("Connecting to %s:%d\n", server_name, server_port);

    struct sockaddr_in server_address;
    memset(&server_address, 0, sizeof(server_address));
    server_address.sin_family = AF_INET;

    char mainString[(int)pow(2,max_message_size_pow)];
    memset(mainString, '*', sizeof(mainString) );

    // creates binary representation of server name
    // and stores it as sin_addr
    // http://beej.us/guide/bgnet/output/html/multipage/inet_ntopman.html
    inet_pton(AF_INET, server_name, &server_address.sin_addr);

    // htons: port in network order format
    server_address.sin_port = htons(server_port);

    // open a stream socket
    if ((sock = socket(PF_INET, SOCK_STREAM, 0)) < 0) {
        printf("could not create socket: %s\n", strerror(errno));
        return 1;
    }

    // TCP is connection oriented, a reliable connection
    // **must** be established before any data is exchanged
    if (connect(sock, (struct sockaddr*)&server_address,
                sizeof(server_address)) < 0) {
        printf("could not connect to server: %s\n", strerror(errno));
        return 1;
    }

    // send

    int opt = 1;
    setsockopt( sock, IPPROTO_TCP, TCP_NODELAY, (void *)&opt, sizeof(opt));


    /* For printing progress in a single line with dots */
    setbuf(stdout, NULL);


    bool success = runTests("tcp", &send_data, NULL, mainString);

    // close the socket
    close(sock);
    return 0;
}
