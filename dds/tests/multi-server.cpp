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


using namespace std;

using namespace eprosima::fastrtps;
using namespace eprosima::fastrtps::rtps;

bool running = true;

char *topicSelf, *topicForward;

static void stopHandler(int sig) {
    printf("received ctrl-c\n");
    running = false;
}


eprosima::fastrtps::Publisher* echo_publisher;

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
                if (!echo_publisher->write((void*)&pubMessage)) {
                    std::cerr << "Could not write echo reply" << std::endl;
                } else {
                    //std::cout << "Sent echo reply" << std::endl;
                }
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

    if (argc != 3) {
        printf("Usage: program TOPIC_SELF TOPIC_FORWARD\n");
        return 1;
    }

    topicSelf = argv[1];
    topicForward = argv[2];

    char clientId[255];
    snprintf(clientId, 255, "client_%s", topicSelf);


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
    PParam.rtps.setName(clientId);
    mp_participant = Domain::createParticipant(PParam);

    if(mp_participant==nullptr)
        return 1;

    //REGISTER THE TYPE
    DDSPerfTest::PubMessagePubSubType m_type_pub;
    Domain::registerType(mp_participant,&m_type_pub);

    //CREATE THE ECHO PUBLISHER
    PublisherAttributes echo_write_param;
    echo_write_param.topic.topicKind = NO_KEY;
    echo_write_param.topic.topicDataType = "DDSPerfTest::PubMessage";
    echo_write_param.topic.topicName = topicForward;
    echo_write_param.topic.historyQos.kind = KEEP_LAST_HISTORY_QOS;
    echo_write_param.topic.historyQos.depth = 20;
    echo_write_param.topic.resourceLimitsQos.max_samples = 50;
    echo_write_param.topic.resourceLimitsQos.allocated_samples = 50;
    echo_write_param.times.heartbeatPeriod.seconds = 2;
    echo_write_param.times.heartbeatPeriod.fraction = 200*1000*1000;
    echo_write_param.qos.m_reliability.kind = RELIABLE_RELIABILITY_QOS;
    echo_write_param.qos.m_durability.kind = TRANSIENT_LOCAL_DURABILITY_QOS;
    //echo_write_param.qos.m_publishMode.kind = ASYNCHRONOUS_PUBLISH_MODE;
    echo_write_param.historyMemoryPolicy = eprosima::fastrtps::rtps::PREALLOCATED_MEMORY_MODE;
    echo_publisher = Domain::createPublisher(mp_participant,echo_write_param);
    if(echo_publisher == nullptr)
        return 1;

    //CREATE THE SUBSCRIBER
    SubscriberAttributes echo_read_param;
    echo_read_param.topic.topicKind = NO_KEY;
    echo_read_param.topic.topicDataType = "DDSPerfTest::PubMessage";
    echo_read_param.topic.topicName = topicSelf;
    echo_read_param.topic.historyQos.kind = KEEP_LAST_HISTORY_QOS;
    echo_read_param.topic.historyQos.depth = 20;
    echo_read_param.topic.resourceLimitsQos.max_samples = 50;
    echo_read_param.topic.resourceLimitsQos.allocated_samples = 50;
    echo_read_param.qos.m_reliability.kind = RELIABLE_RELIABILITY_QOS;
    //echo_read_param.qos.m_durability.kind = VOLATILE_DURABILITY_QOS;
    echo_read_param.qos.m_durability.kind = TRANSIENT_LOCAL_DURABILITY_QOS;
    echo_read_param.historyMemoryPolicy = eprosima::fastrtps::rtps::PREALLOCATED_MEMORY_MODE;

    EchoReplyListener echoReplyListener;
    eprosima::fastrtps::Subscriber* echo_subscriber = Domain::createSubscriber(mp_participant,echo_read_param,(SubscriberListener*)&echoReplyListener);

    if(echo_subscriber == nullptr)
        return 1;


    while(running) {
        usleep(1000);
    }


    return 0;
}
