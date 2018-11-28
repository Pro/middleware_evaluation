//
// Created by profanter on 9/4/18.
// Copyright (c) 2018 fortiss GmbH. All rights reserved.
//

#ifndef PROJECT_TEST_WRAPPER_H
#define PROJECT_TEST_WRAPPER_H

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <sys/types.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>

#include "time_utils.h"

#define REPETITIONS_RANDOM 100 // 100
#define REPETITIONS_ECHO 5000 // 5000
#define REPETITIONS_ACK 5000  // 5000
#define max_message_size_pow 15 // 20 // maximum message size by power of 2

int currentProgressPos = 0;

void printProgressStart() {
    printf("\n");
    currentProgressPos = 0;
}

void printProgress(int test, int count) {
    int barWidth = 70;
    float progress = (float)test / (float)count;

    int pos = barWidth * progress;
    if (pos == currentProgressPos)
        return;
    currentProgressPos = pos;
    printf("[");
    for (int i = 0; i < barWidth; ++i) {
        if (i < pos) printf("=");
        else if (i == pos) printf(">");
        else printf(" ");
    }
    printf("] %d%%  (%d/%d)\r", (int)(progress * 100.0), test, count);
}

void printProgressEnd() {
    printf("\n");
}

bool runTests(const char* filePrefix, bool (*performCall)(void*,void*,size_t, long long int*,bool), void* context, void* payload) {

    if (performCall == NULL)
        return false;

    //int num_reqs;
    long long int total_time;
    double rtt;
    /* ----------------------------------------------------------
                Testing for non-homogeneous workloads
       ---------------------------------------------------------- */

    long long int duration_random[REPETITIONS_RANDOM];
    long long int duration_echo[max_message_size_pow+1][REPETITIONS_ECHO];
    long long int duration_ack[max_message_size_pow+1][REPETITIONS_ACK];

    struct timespec wait_time;
    total_time = 0;
    srand (1); // initialize random seed, to generate same random behavior on all tests
    printf("Testing random repetitions");
    printProgressStart();
    for(unsigned int t=0; t<REPETITIONS_RANDOM; t++){
        printProgress(t, REPETITIONS_RANDOM);
        // wait between 0-5 seconds
        wait_time.tv_sec = rand() % 2;
        wait_time.tv_nsec = rand() % 1000000000;
        nanosleep(&wait_time, NULL);
        if (!performCall(context, payload, 4, &(duration_random[t]), true))
            return false;
        total_time += duration_random[t];
    }
    printProgress(REPETITIONS_RANDOM, REPETITIONS_RANDOM);
    printProgressEnd();
    rtt = total_time / 1000.0 / REPETITIONS_RANDOM;
    printf("\nvariable requests average rtt/request=%.2f\n\n", rtt);
    // ----------------------------------------------------------

    /* ----------------------------------------------------------
                Testing throughput when echoing
       ---------------------------------------------------------- */
    for(size_t power=1; power <= max_message_size_pow; power++){
        int str_len = (int)pow(2, power);
        total_time = 0;
        printf("Testing echoing with %d bytes", str_len);
        printProgressStart();
        for(unsigned int t=0; t<REPETITIONS_ECHO; t++){
            printProgress(t, REPETITIONS_ECHO);
            if (!performCall(context, payload, str_len, &(duration_echo[power-1][t]), true))
                return false;
            total_time += duration_echo[power-1][t];
            //printf("Test [%d-%d] %lld\n", str_len, t, duration_echo[power-1][t]);
        }
        printProgress(REPETITIONS_ECHO, REPETITIONS_ECHO);
        printProgressEnd();
        rtt = total_time / 1000.0 / REPETITIONS_ECHO;
        printf("echo_str %6d average rtt/request=%.2f\n",           str_len, rtt);
        printf("echo_str %6d messages/s=%.2f\n",                    str_len, 1000.0 / rtt);
        printf("echo_str %6d %.2f kB/s (bidirectional sum)\n\n",    str_len, 2.0 * str_len / rtt);
    }
    printProgressEnd();
    // ----------------------------------------------------------

    /* ----------------------------------------------------------
                Testing throughput without echoing
       ---------------------------------------------------------- */
    for(size_t power=1; power <= max_message_size_pow; power++){
        int str_len = (int)pow(2, power);
        total_time = 0;
        printf("Testing acking with %d bytes", str_len);
        printProgressStart();
        for(unsigned int t=0; t<REPETITIONS_ACK; t++){
            printProgress(t, REPETITIONS_ACK);
            if (!performCall(context, payload, str_len, &(duration_ack[power-1][t]), false))
                return false;
            total_time += duration_ack[power-1][t];
        }
        printProgress(REPETITIONS_ACK, REPETITIONS_ACK);
        printProgressEnd();
        rtt = total_time / 1000.0 / REPETITIONS_ACK;
        printf("ack_str %6d average rtt/request=%.2f\n",           str_len, rtt);
        printf("ack_str %6d messages/s=%.2f\n",                    str_len, 1000.0 / rtt);
        printf("ack_str %6d %.2f kB/s (bidirectional sum)\n\n",    str_len, 2.0 * str_len / rtt);
    }
    printProgressEnd();
    // ----------------------------------------------------------


    struct tm *timenow;
    time_t now = time(NULL);
    timenow = gmtime(&now);

    char filePrefixDate[255];
    strftime(filePrefixDate, 255, "%Y-%m-%d_%H%M%S", timenow);

    char filename[255];
    snprintf(filename, 255, "%s-%s_random.csv", filePrefix, filePrefixDate);

    FILE *fp=fopen(filename,"w");
    if (fp == NULL) {
        perror("Error opening file for random results");
    } else {
        fprintf(fp,"Test Number, Duration [usec]");
        for (unsigned int i=0; i<REPETITIONS_RANDOM; i++) {
            if (fprintf(fp, "\n%d, %lld", i, duration_random[i]) <= 0) {
                perror("Error writing to random results file");
                break;
            }
        }
    }
    fclose (fp);
    printf("Written results to %s\n", filename);

    snprintf(filename, 255, "%s-%s_echo.csv", filePrefix, filePrefixDate);
    fp=fopen(filename,"w");
    if (fp == NULL) {
        perror("Error opening file for echo results");
    } else {
        fprintf(fp,"Test Number, Payload Size [bytes], Duration [usec]");
        for(size_t power=1; power <= max_message_size_pow; power++) {
            int str_len = (int) pow(2, power);
            for (unsigned int t = 0; t < REPETITIONS_ECHO; t++) {
                if (fprintf(fp, "\n%d, %d, %lld", t, str_len, duration_echo[power-1][t]) <= 0) {
                    perror("Error writing to echo results file");
                    break;
                }
            }
        }
    }
    fclose (fp);
    printf("Written results to %s\n", filename);

    snprintf(filename, 255, "%s-%s_ack.csv", filePrefix, filePrefixDate);
    fp=fopen(filename,"w");
    if (fp == NULL) {
        perror("Error opening file for ack results");
    } else {
        fprintf(fp,"Test Number, Payload Size [bytes], Duration [usec]");
        for(size_t power=1; power <= max_message_size_pow; power++) {
            int str_len = (int) pow(2, power);
            for (unsigned int t = 0; t < REPETITIONS_ACK; t++) {
                if (fprintf(fp, "\n%d, %d, %lld", t, str_len, duration_ack[power-1][t]) <= 0) {
                    perror("Error writing to ack results file");
                    break;
                }
            }
        }
    }
    fclose (fp);
    printf("Written results to %s\n", filename);

    return true;
}

#endif //PROJECT_TEST_WRAPPER_H
