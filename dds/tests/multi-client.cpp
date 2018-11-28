//
// Created by profanter on 9/11/18.
// Copyright (c) 2018 fortiss GmbH. All rights reserved.
//

#include <fastrtps/fastrtps_fwd.h>
#include <fastrtps/attributes/PublisherAttributes.h>
#include <fastrtps/publisher/PublisherListener.h>
#include <fastrtps/participant/Participant.h>
#include <fastrtps/attributes/ParticipantAttributes.h>
#include <fastrtps/attributes/PublisherAttributes.h>
#include <fastrtps/publisher/Publisher.h>
#include <fastrtps/attributes/SubscriberAttributes.h>
#include <fastrtps/subscriber/SubscriberListener.h>
#include <fastrtps/subscriber/SampleInfo.h>
#include <fastrtps/attributes/SubscriberAttributes.h>
#include <fastrtps/subscriber/Subscriber.h>
#include <fastrtps/Domain.h>
#include <fastrtps/utils/eClock.h>
#include <fastrtps/log/StdoutConsumer.h>

#include <DDSPerfTestPubSubTypes.h>

#include <cstdio>
#include <csignal>
#include <unistd.h>

#include "common.h"

using namespace eprosima::fastrtps;
using namespace eprosima::fastrtps::rtps;


#define PAYLOAD_SIZE 10240
#define PARALLEL_FORWARD 10
#define RUNS 100


uint64_t get_microseconds() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
    return ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
}

char *topicSelf;
uint64_t dataReceived = 0;

bool running = true;
static void stopHandler(int sig) {
    printf("received ctrl-c\n");
    running = false;
}


class EchoReplyListener:public eprosima::fastrtps::SubscriberListener
{
public:
    EchoReplyListener(){};
    ~EchoReplyListener(){};
    void onSubscriptionMatched(eprosima::fastrtps::Subscriber* sub, eprosima::fastrtps::rtps::MatchingInfo& info)
    {
        if(info.status == MATCHED_MATCHING)
        {
            std::cout << "Echo Subscriber matched"<<std::endl;
        }
        else
        {
            std::cout << "Echo Subscriber unmatched"<<std::endl;
        }
    }

    void onNewDataMessage(eprosima::fastrtps::Subscriber* sub)
    {

        if(sub->takeNextData((void*)&pubMessage, &m_info))
        {
            if(m_info.sampleKind == ALIVE)
            {
                dataReceived += pubMessage.buffer().size();
            }
        }

    }

    DDSPerfTest::PubMessage pubMessage;
    eprosima::fastrtps::SampleInfo_t m_info;
};


int main(int argc, char **argv)
{

    signal(SIGINT, stopHandler);
    signal(SIGTERM, stopHandler);

    if (argc != 4) {
        printf("Usage: program TOPIC_SELF TOPIC_FORWARD_PREFIX TOPIC_FORWARD_IDX\n");
        return 1;
    }

    topicSelf = argv[1];
    char *topicForwardPrefix = argv[2];
    int topicForwardStart = atoi(argv[3]);

    char clientId[255];
    snprintf(clientId, 255, "client_%s", topicSelf);

    char topicNames[PARALLEL_FORWARD][255];
    for (unsigned int i = 0; i < PARALLEL_FORWARD; i++) {
        snprintf(topicNames[i], 255, "%s%d", topicForwardPrefix, topicForwardStart + i);

    }


    eprosima::fastrtps::Participant* mp_participant;

    std::unique_ptr<StdoutConsumer> consumer(new StdoutConsumer);
    Log::RegisterConsumer(std::move(consumer));
    Log::SetVerbosity(Log::Info);

    ParticipantAttributes PParam;
    PParam.rtps.defaultSendPort = 11511;
    PParam.rtps.builtin.use_SIMPLE_RTPSParticipantDiscoveryProtocol = true;
    PParam.rtps.builtin.use_SIMPLE_EndpointDiscoveryProtocol = true;
    PParam.rtps.builtin.m_simpleEDP.use_PublicationReaderANDSubscriptionWriter = true;
    PParam.rtps.builtin.m_simpleEDP.use_PublicationWriterANDSubscriptionReader = true;
    PParam.rtps.builtin.domainId = 111;
    PParam.rtps.builtin.leaseDuration = c_TimeInfinite;
    PParam.rtps.sendSocketBufferSize = 65536;
    PParam.rtps.listenSocketBufferSize = 2*65536;
    PParam.rtps.setName("Participant_client");
    mp_participant = Domain::createParticipant(PParam);

    if(mp_participant==nullptr)
        return 1;

    //REGISTER THE TYPE
    DDSPerfTest::PubMessagePubSubType m_type_pub;
    Domain::registerType(mp_participant,&m_type_pub);

    //CREATE THE SUBSCRIBER
    SubscriberAttributes echo_read_param;
    echo_read_param.topic.topicKind = NO_KEY;
    echo_read_param.topic.topicDataType = "DDSPerfTest::PubMessage";
    echo_read_param.topic.topicName = topicSelf;
    echo_read_param.topic.historyQos.kind = KEEP_LAST_HISTORY_QOS;
    echo_read_param.topic.historyQos.depth = 30;
    echo_read_param.topic.resourceLimitsQos.max_samples = 50;
    echo_read_param.topic.resourceLimitsQos.allocated_samples = 50;
    echo_read_param.qos.m_reliability.kind = RELIABLE_RELIABILITY_QOS;
    echo_read_param.qos.m_durability.kind = VOLATILE_DURABILITY_QOS;
    echo_read_param.historyMemoryPolicy = eprosima::fastrtps::rtps::PREALLOCATED_MEMORY_MODE;

    EchoReplyListener echoReplyListener;
    eprosima::fastrtps::Subscriber* echo_subscriber = Domain::createSubscriber(mp_participant,echo_read_param,(SubscriberListener*)&echoReplyListener);

    if(echo_subscriber == nullptr)
        return 1;

    eprosima::fastrtps::Publisher* echo_publisher[PARALLEL_FORWARD];
    for (unsigned int i = 0; i < PARALLEL_FORWARD && running; i++) {



        //CREATE THE ECHO PUBLISHER
        PublisherAttributes echo_write_param;
        echo_write_param.topic.topicKind = NO_KEY;
        echo_write_param.topic.topicDataType = "DDSPerfTest::PubMessage";
        echo_write_param.topic.topicName = topicNames[i];
        echo_write_param.topic.historyQos.kind = KEEP_LAST_HISTORY_QOS;
        echo_write_param.topic.historyQos.depth = 30;
        echo_write_param.topic.resourceLimitsQos.max_samples = 50;
        echo_write_param.topic.resourceLimitsQos.allocated_samples = 50;
        echo_write_param.times.heartbeatPeriod.seconds = 2;
        echo_write_param.times.heartbeatPeriod.fraction = 200*1000*1000;
        echo_write_param.qos.m_reliability.kind = RELIABLE_RELIABILITY_QOS;
        //echo_write_param.qos.m_publishMode.kind = ASYNCHRONOUS_PUBLISH_MODE;
        echo_write_param.historyMemoryPolicy = eprosima::fastrtps::rtps::PREALLOCATED_MEMORY_MODE;
        echo_publisher[i] = Domain::createPublisher(mp_participant,echo_write_param);
        if(echo_publisher[i] == nullptr)
            return 1;

    }
    DDSPerfTest::PubMessage pubmessage_data;
    pubmessage_data.buffer().resize(PAYLOAD_SIZE, '*');


    sleep(4);

    uint64_t elapsedTime[RUNS];
    for (unsigned int k = 0; k < RUNS && running; k++) {
        dataReceived = 0;


        printf("[%d/%d] Publishing data\n", k, RUNS);
        uint64_t start_time = get_microseconds();
        for (unsigned int i = 0; i < PARALLEL_FORWARD; i++) {

            if (!echo_publisher[i]->write((void*)&pubmessage_data)) {
                std::cerr << "Could not write echo message" << std::endl;
                return false;
            } else {
                //std::cout << "Sent echo reply" << std::endl;
            }
        }

        while (running && dataReceived != PARALLEL_FORWARD * PAYLOAD_SIZE) {
            usleep(10);
        }
        elapsedTime[k] = get_microseconds() - start_time;
        printf("[%d/%d] All data received.\n", k, RUNS);
    }

    if (running) {

        uint64_t totalTime = 0;
        printf("-------------\nNode;Microseconds\n");
        for (unsigned int k = 0; k < RUNS; k++) {
            printf("%d;%ld\n",k, elapsedTime[k]);
            if (k > 0) // skip first, since it also includes setting up connection
                totalTime += elapsedTime[k];
        }
        printf("---------------\n");
        printf("Total Elapsed = %fms, Average = %fms\n", totalTime/1000.0, (totalTime / (double)(RUNS-1))/1000.0);

    }

    // bye bye

    Domain::removeParticipant(mp_participant);

    return 0;
}