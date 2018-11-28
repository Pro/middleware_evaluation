//
// Created by profanter on 9/3/18.
// Copyright (c) 2018 fortiss GmbH. All rights reserved.
//

#ifndef PROJECT_PROC_STAT_H
#define PROJECT_PROC_STAT_H

#include <sys/types.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/time.h>
#include <sys/resource.h>


int setProcPriority() {

    if (setpriority(PRIO_PROCESS, 0, -20) == -1) {
        fprintf(stderr, "WARNING: Could not set process priority");
    };

#if (defined (MCL_CURRENT) && defined(MCL_FUTURE) && !defined(__ANDROID__))
    if (mlockall(MCL_CURRENT | MCL_FUTURE)) {
        fprintf(stderr, "WARNING: Could not lock memory - Run with root access.\n");
    }
#endif
}

int _procToCsv(const char *option, const char* outputFile) {

    pid_t pidSelf = getpid();

    const char* procstat = getenv("PROCSTAT");
    if (procstat == NULL || access( procstat, F_OK ) == -1) {
        fprintf(stderr, "Environment variable PROCSTAT must point to the linuxprocfs.py script.\n");
        return -1;
    }

    if (access( outputFile, F_OK ) != -1) {
        fprintf(stderr, "Output file '%s' already exists.", outputFile);
        return -1;
    }
    char cmd[255];

    snprintf(cmd, 255, "%s -w1 %s -t -r %d > %s & echo $!", procstat, option, pidSelf, outputFile);

    FILE *p = popen(cmd, "r");
    int pid;
    if (fscanf(p, "%d", &pid) <= 0) {
        fprintf(stderr, "Cannot get PID for PROCSTAT process");
        return -1;
    }
    pclose(p);

    // wait a bit until the proc stat is up and running
    sleep(1);
    return pid;
}

int runProcStatToCsv(const char *outputFile) {
    return _procToCsv("s", outputFile);
}

int runProcIoToCsv(const char *outputFile) {
    return _procToCsv("i", outputFile);
}

void stopProcToCsv(int pid) {
    char cmd[255];
    snprintf(cmd, 255, "kill -2 %d", pid);
    if (system(cmd) == -1) {
        fprintf(stderr, "Cannot get stop PROCSTAT process");
    }
}


#endif //PROJECT_PROC_STAT_H
