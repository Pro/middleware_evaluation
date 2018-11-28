//
// Created by profanter on 9/6/18.
// Copyright (c) 2018 fortiss GmbH. All rights reserved.
//

#ifndef PROJECT_COMMON_H
#define PROJECT_COMMON_H

#define TOPIC_ACK_SEND "ack_send"
#define TOPIC_ACK_REC "ack_rec"
#define TOPIC_ECHO_SEND "echo_send"
#define TOPIC_ECHO_REC "echo_rec"

/* void set_rt() */
/*      Attempt to set the real time priority and lock memory */
void set_rt() {
    ACE_Sched_Params params(ACE_SCHED_FIFO,
                            ACE_DEFAULT_THREAD_PRIORITY,
                            ACE_SCOPE_PROCESS);

#if defined (ACE_HAS_WTHREADS)
    params.priority(THREAD_PRIORITY_HIGHEST);
#else
    params.priority(20);
#endif

    if (-1 == ACE_OS::sched_params(params)) {
        ACE_DEBUG ((LM_DEBUG, "WARNING: %p\n", ACE_TEXT("sched_params")));
    }

#if (defined (MCL_CURRENT) && defined(MCL_FUTURE) && !defined(__ANDROID__))
    if (mlockall(MCL_CURRENT | MCL_FUTURE)) {
        ACE_DEBUG ((LM_DEBUG, "WARNING: Could not lock memory - Run with root access.\n"));
    }
#endif
}

#endif //PROJECT_COMMON_H
