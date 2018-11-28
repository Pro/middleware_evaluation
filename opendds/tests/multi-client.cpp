//
// Created by profanter on 9/1/18.
// Copyright (c) 2018 fortiss GmbH. All rights reserved.
//

#include "EchoDataReader.h"
#include "DDSPerfTestTypeSupportImpl.h"
#include <dds/DCPS/Service_Participant.h>
#include <dds/DCPS/Marked_Default_Qos.h>
#include <dds/DCPS/PublisherImpl.h>
#include <dds/DCPS/SubscriberImpl.h>
#include <dds/DCPS/transport/framework/TransportRegistry.h>
#include <dds/DCPS/transport/framework/TransportExceptions.h>
#include <ace/streams.h>

#include "ace/Get_Opt.h"
#include "ace/Sched_Params.h"
#include "ace/OS_NS_unistd.h"

#include <iostream>
#include <cstdio>

#include "common.h"

using namespace DDS;
using namespace CORBA;
using namespace DDSPerfTest;

#define PAYLOAD_SIZE 10240
#define PARALLEL_FORWARD 10
#define RUNS 100


uint64_t get_microseconds() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
    return ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
}

long dataReceived = 0;
void onMessageData(PubMessage *msg) {
    dataReceived += msg->buffer.length();
}


bool running = true;

static void stopHandler(int sig) {
    running = false;
}

int ACE_TMAIN(int argc, ACE_TCHAR *argv[]) {


    signal(SIGINT, stopHandler);
    signal(SIGTERM, stopHandler);

    try {
        DDS::DomainParticipantFactory_var dpf =
                TheParticipantFactoryWithArgs(argc, argv);


        std::setbuf(stdout, NULL);

        /* Try to set realtime scheduling class*/
        set_rt();

        DDS::DomainParticipant_var dp =
                dpf->create_participant(111,
                                        PARTICIPANT_QOS_DEFAULT,
                                        DDS::DomainParticipantListener::_nil(),
                                        ::OpenDDS::DCPS::DEFAULT_STATUS_MASK);
        if (CORBA::is_nil(dp.in())) {
            cerr << "create_participant failed." << endl;
            return 1;
        }

        std::string topicSelf, topicForwardPrefix;
        int topicForwardStart;

        ACE_Get_Opt get_opts(argc, argv, ACE_TEXT("s:f:i:"));
        int ich;
        while ((ich = get_opts()) != EOF) {
            switch (ich) {

                case 's': /* self topic */
                    topicSelf = std::string(get_opts.opt_arg());
                    break;
                case 'f': /* forward topic */
                    topicForwardPrefix = std::string(get_opts.opt_arg());
                    break;

                case 'i': /* forward topic */
                    topicForwardStart = ACE_OS::atoi(get_opts.opt_arg());

                    if (topicForwardStart < 0) {
                        std::cerr << "sample_pub: ERROR - forward index\n";
                        exit(1);
                    }

                    break;

                default: /* no parameters */
                    break;

            }
        }

        if (topicSelf.length() == 0 || topicForwardPrefix.length() == 0) {
            cerr << "Usage: program -s TOPIC_SELF -f TOPIC_FORWARD_PREFIX -i TOPIC_FORWARD_IDX" << endl;
            exit(1);
        }


        char topicNames[PARALLEL_FORWARD][255];
        for (unsigned int i = 0; i < PARALLEL_FORWARD; i++) {
            snprintf(topicNames[i], 255, "%s%d", topicForwardPrefix.c_str(), topicForwardStart + i);

        }


        DDS::TopicQos topicQos;
        dp->get_default_topic_qos(topicQos);
        topicQos.resource_limits.max_instances = 1;
        topicQos.resource_limits.max_samples = 1;
        topicQos.reliability.kind = DDS::BEST_EFFORT_RELIABILITY_QOS;

        /**
         * ECHO Subscriber
         */

        /* Create the subscriber */
        DDS::Subscriber_var echo_sub =
                dp->create_subscriber(SUBSCRIBER_QOS_DEFAULT,
                                      DDS::SubscriberListener::_nil(),
                                      ::OpenDDS::DCPS::DEFAULT_STATUS_MASK);

        /* Create topic for datareader */
        PubMessageTypeSupportImpl *echomessage_dt = new PubMessageTypeSupportImpl;
        DDS::ReturnCode_t register_status = echomessage_dt->register_type(dp.in(),
                                                                          "DDSPerfTest::PubMessage");
        if (register_status != DDS::RETCODE_OK) {
            std::cerr << "ERROR: sample_pub failed to register PubMessage with return code "
                      << register_status << std::endl;
            exit(1);
        }

        DDS::Topic_var echo_rec_topic = dp->create_topic(topicSelf.c_str(), // topic name
                                                         "DDSPerfTest::PubMessage", // topic type
                                                         topicQos,
                                                         DDS::TopicListener::_nil(),
                                                         ::OpenDDS::DCPS::DEFAULT_STATUS_MASK);
        DDS::DataReaderQos echoReaderQos;
        echo_sub->get_default_datareader_qos(echoReaderQos);
        echoReaderQos.resource_limits.max_instances = 1;
        echoReaderQos.resource_limits.max_samples = 1;
        echoReaderQos.reliability.kind = DDS::BEST_EFFORT_RELIABILITY_QOS;




        /* Create the listener for datareader */
        DDS::DataReaderListener_var echo_listener(new EchoDataReaderImpl());
        EchoDataReaderImpl *echo_listener_servant =
                dynamic_cast<EchoDataReaderImpl *>(echo_listener.in());


        /* Create PubMessage datareader */
        DDS::DataReader_var echo_dr = echo_sub->create_datareader(echo_rec_topic.in(),
                                                                  echoReaderQos,
                                                                  echo_listener.in(),
                                                                  ::OpenDDS::DCPS::DEFAULT_STATUS_MASK);

        echo_listener_servant->init(echo_dr.in(), true);
        echo_listener_servant->setDataCallback(&onMessageData);


        PubMessageDataWriter_var messageWriter[PARALLEL_FORWARD];
        InstanceHandle_t instanceHandle[PARALLEL_FORWARD];

        srand((unsigned int) time(NULL));

        PubMessage message;
        message.buffer.length(PAYLOAD_SIZE);// let's try with a workload that is ignorable

        for (int i = 0; i < message.buffer.length(); i++)
            message.buffer[i] = (char) (rand() % 255);

        for (unsigned int i = 0; i < PARALLEL_FORWARD && running; i++) {
            DDS::Topic_var topic_echo =
                    dp->create_topic(topicNames[i],
                                     "DDSPerfTest::PubMessage",
                                     topicQos,
                                     DDS::TopicListener::_nil(),
                                     ::OpenDDS::DCPS::DEFAULT_STATUS_MASK);
            if (CORBA::is_nil(topic_echo.in())) {
                cerr << "create_topic failed." << endl;
                exit(1);
            }

            /* Create publisher */
            DDS::Publisher_var pub_echo =
                    dp->create_publisher(PUBLISHER_QOS_DEFAULT,
                                         DDS::PublisherListener::_nil(),
                                         ::OpenDDS::DCPS::DEFAULT_STATUS_MASK);
            if (CORBA::is_nil(pub_echo.in())) {
                cerr << "create_publisher failed." << endl;
                exit(1);
            }

            DDS::DataWriterQos echoWriterQos;
            pub_echo->get_default_datawriter_qos(echoWriterQos);
            echoWriterQos.resource_limits.max_instances = 1;
            echoWriterQos.resource_limits.max_samples = 1;
            echoWriterQos.reliability.kind = DDS::BEST_EFFORT_RELIABILITY_QOS;

            /* Create PubMessage datawriter */
            DDS::DataWriter_var echo_dw = pub_echo->create_datawriter(topic_echo.in(),
                                                                      echoWriterQos,
                                                                      DDS::DataWriterListener::_nil(),
                                                                      ::OpenDDS::DCPS::DEFAULT_STATUS_MASK);
            messageWriter[i] = PubMessageDataWriter::_narrow(echo_dw.in());

            if (CORBA::is_nil(messageWriter[i].in())) {
                cerr << "Data Writer could not be narrowed" << endl;
                exit(1);
            }
            instanceHandle[i] = messageWriter[i]->register_instance(message);
        }

        uint64_t elapsedTime[RUNS];
        for (unsigned int k = 0; k < RUNS && running; k++) {

            bool timeout = false;

            uint64_t start_time;
            do {
                timeout = false;
                dataReceived = 0;

                printf("[%d/%d] Publishing data\n", k, RUNS);
                start_time = get_microseconds();
                for (unsigned int i = 0; i < PARALLEL_FORWARD; i++) {
                    ::DDS::ReturnCode_t ret;
                    ret = messageWriter[i]->write(message, instanceHandle[i]);

                    if (ret != ::DDS::RETCODE_OK) {
                        ACE_ERROR ((LM_ERROR,
                                ACE_TEXT("(%P|%t) ERROR: Writer::svc, ")
                                           ACE_TEXT("write() returned %d.\n"),
                                                   ret));
                        continue;
                    }
                }

                while (running && dataReceived < PARALLEL_FORWARD * PAYLOAD_SIZE && !timeout) {
                    ACE_OS::sleep(ACE_Time_Value(0, 10));

                    timeout = (get_microseconds() - start_time) > 5000000;
                }
                if (timeout)
                    printf("Timeout. Retrying...\n");
            } while(timeout);

            elapsedTime[k] = get_microseconds() - start_time;
            printf("[%d/%d] All data received.\n", k, RUNS);


        }


        if (running) {

            uint64_t totalTime = 0;
            printf("-------------\nNode;Microseconds\n");
            for (unsigned int k = 0; k < RUNS; k++) {
                printf("%d;%ld\n", k, elapsedTime[k]);
                if (k > 0) // skip first, since it also includes setting up connection
                    totalTime += elapsedTime[k];
            }
            printf("---------------\n");
            printf("Total Elapsed = %fms, Average = %fms\n", totalTime / 1000.0,
                   (totalTime / (double) (RUNS - 1)) / 1000.0);

        }



        dp->delete_contained_entities();
        dpf->delete_participant(dp.in());
        TheServiceParticipant->shutdown();
    }
    catch (CORBA::Exception &e) {
        cerr << "PUB: Exception caught in main.cpp:" << endl
             << e << endl;
        exit(1);
    }

    return 0;
}
