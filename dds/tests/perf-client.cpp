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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <math.h>
#include <unistd.h>
#include "time_utils.h"
#include "test_wrapper.h"

#include "common.h"

using namespace eprosima::fastrtps;
using namespace eprosima::fastrtps::rtps;

bool echoReceived = false;
bool ackReceived = false;

class AckReplyListener:public eprosima::fastrtps::SubscriberListener
{
public:
    AckReplyListener(){};
    ~AckReplyListener(){};
    void onSubscriptionMatched(eprosima::fastrtps::Subscriber* sub, eprosima::fastrtps::rtps::MatchingInfo& info)
    {
        if(info.status == MATCHED_MATCHING)
        {
            std::cout << "ACK Subscriber matched"<<std::endl;
        }
        else
        {
            std::cout << "ACK Subscriber unmatched"<<std::endl;
        }
    }

    void onNewDataMessage(eprosima::fastrtps::Subscriber* sub)
    {

        if(sub->takeNextData((void*)&pubMessage, &m_info))
        {
            if(m_info.sampleKind == ALIVE)
            {
                ackReceived = true;
            }
        }

    }
    DDSPerfTest::AckMessage ackMessage;
    DDSPerfTest::PubMessage pubMessage;
    eprosima::fastrtps::SampleInfo_t m_info;
};

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
                echoReceived = true;
            }
        }

    }

    DDSPerfTest::PubMessage pubMessage;
    eprosima::fastrtps::SampleInfo_t m_info;
};


DDSPerfTest::PubMessage pubmessage_data;


eprosima::fastrtps::Publisher* ack_publisher;
eprosima::fastrtps::Publisher* echo_publisher;

bool sendMessage(void *context, void *payloadData, size_t payloadSize, long long int *elapsedTime, bool isEcho) {

    if (payloadSize != pubmessage_data.buffer().size()) {

        // maximum size we are using
        pubmessage_data.buffer().resize(payloadSize, '*');
    }



    bool success = false;
    do {


        long count = 0;
        long sleepUsec = 10;
        long countTimeout = 10000000/sleepUsec;

        struct timespec time_start;
        TIME_MEASURE_START(time_start);
        if (isEcho) {
            echoReceived = false;
            if (!echo_publisher->write((void*)&pubmessage_data)) {
                std::cerr << "Could not write echo message" << std::endl;
                sleep(1);
                continue;
            }
            while (!echoReceived && count < countTimeout) {
                usleep(sleepUsec);
                count ++;
            }
            success = echoReceived;
        } else {
            ackReceived = false;
            if (!ack_publisher->write((void*)&pubmessage_data)) {
                std::cerr << "Could not write ack message" << std::endl;
                sleep(1);
                continue;
            }
            while (!ackReceived && count < countTimeout) {
                usleep(sleepUsec);
                count ++;
            }
            success = ackReceived;
        }
        TIME_MEASURE_DIFF_USEC(time_start, *elapsedTime);
        if (!success)
            printf("Timeout. Resending ...");
    } while (!success);

    return true;
}


int main(int argc, char *argv[]) {
    // intialize the large string that will be used for the function calls and set every char to '*'
    char mainString[(int)pow(2,max_message_size_pow)];
    for(int i = 0; i < sizeof(mainString); i++){mainString[i] = '*';}
    // init. the struct that will be sent to the server, and the variable that will hold the elapsed time for each call

    eprosima::fastrtps::Participant* mp_participant;

    pubmessage_data = DDSPerfTest::PubMessage();

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
    DDSPerfTest::AckMessagePubSubType m_type_ack;
    DDSPerfTest::PubMessagePubSubType m_type_pub;
    Domain::registerType(mp_participant,&m_type_ack);
    Domain::registerType(mp_participant,&m_type_pub);



    //CREATE THE ACK PUBLISHER
    PublisherAttributes ack_write_param;
    ack_write_param.topic.topicKind = NO_KEY;
    ack_write_param.topic.topicDataType = "DDSPerfTest::PubMessage";
    ack_write_param.topic.topicName = TOPIC_ACK_SEND;
    ack_write_param.topic.historyQos.kind = KEEP_LAST_HISTORY_QOS;
    ack_write_param.topic.historyQos.depth = 1;
    ack_write_param.topic.resourceLimitsQos.max_samples = 1;
    ack_write_param.topic.resourceLimitsQos.allocated_samples = 1;
    ack_write_param.times.heartbeatPeriod.seconds = 2;
    ack_write_param.times.heartbeatPeriod.fraction = 200*1000*1000;
    ack_write_param.qos.m_reliability.kind = BEST_EFFORT_RELIABILITY_QOS;
    ack_write_param.qos.m_publishMode.kind = ASYNCHRONOUS_PUBLISH_MODE;
    ack_write_param.historyMemoryPolicy = eprosima::fastrtps::rtps::PREALLOCATED_MEMORY_MODE;
    ack_publisher = Domain::createPublisher(mp_participant,ack_write_param);
    if(ack_publisher == nullptr)
        return 1;

    //CREATE THE SUBSCRIBER
    SubscriberAttributes ack_read_param;
    ack_read_param.topic.topicKind = NO_KEY;
    ack_read_param.topic.topicDataType = "DDSPerfTest::AckMessage";
    ack_read_param.topic.topicName = TOPIC_ACK_REC;
    ack_read_param.topic.historyQos.kind = KEEP_LAST_HISTORY_QOS;
    ack_read_param.topic.historyQos.depth = 1;
    ack_read_param.topic.resourceLimitsQos.max_samples = 1;
    ack_read_param.topic.resourceLimitsQos.allocated_samples = 1;
    ack_read_param.qos.m_reliability.kind = BEST_EFFORT_RELIABILITY_QOS;
    ack_read_param.qos.m_durability.kind = TRANSIENT_LOCAL_DURABILITY_QOS;
    ack_read_param.historyMemoryPolicy = eprosima::fastrtps::rtps::PREALLOCATED_MEMORY_MODE;

    AckReplyListener ackReplyListener;
    eprosima::fastrtps::Subscriber* ack_subscriber = Domain::createSubscriber(mp_participant,ack_read_param,(SubscriberListener*)&ackReplyListener);

    if(ack_subscriber == nullptr)
        return 1;

    //CREATE THE ECHO PUBLISHER
    PublisherAttributes echo_write_param;
    echo_write_param.topic.topicKind = NO_KEY;
    echo_write_param.topic.topicDataType = "DDSPerfTest::PubMessage";
    echo_write_param.topic.topicName = TOPIC_ECHO_SEND;
    echo_write_param.topic.historyQos.kind = KEEP_LAST_HISTORY_QOS;
    echo_write_param.topic.historyQos.depth = 1;
    echo_write_param.topic.resourceLimitsQos.max_samples = 1;
    echo_write_param.topic.resourceLimitsQos.allocated_samples = 1;
    echo_write_param.times.heartbeatPeriod.seconds = 2;
    echo_write_param.times.heartbeatPeriod.fraction = 200*1000*1000;
    echo_write_param.qos.m_reliability.kind = BEST_EFFORT_RELIABILITY_QOS;
    echo_write_param.qos.m_publishMode.kind = ASYNCHRONOUS_PUBLISH_MODE;
    echo_write_param.historyMemoryPolicy = eprosima::fastrtps::rtps::PREALLOCATED_MEMORY_MODE;
    echo_publisher = Domain::createPublisher(mp_participant,echo_write_param);
    if(echo_publisher == nullptr)
        return 1;

    //CREATE THE SUBSCRIBER
    SubscriberAttributes echo_read_param;
    echo_read_param.topic.topicKind = NO_KEY;
    echo_read_param.topic.topicDataType = "DDSPerfTest::PubMessage";
    echo_read_param.topic.topicName = TOPIC_ECHO_REC;
    echo_read_param.topic.historyQos.kind = KEEP_LAST_HISTORY_QOS;
    echo_read_param.topic.historyQos.depth = 1;
    echo_read_param.topic.resourceLimitsQos.max_samples = 1;
    echo_read_param.topic.resourceLimitsQos.allocated_samples = 1;
    echo_read_param.qos.m_reliability.kind = BEST_EFFORT_RELIABILITY_QOS;
    echo_read_param.qos.m_durability.kind = TRANSIENT_LOCAL_DURABILITY_QOS;
    echo_read_param.historyMemoryPolicy = eprosima::fastrtps::rtps::PREALLOCATED_MEMORY_MODE;

    EchoReplyListener echoReplyListener;
    eprosima::fastrtps::Subscriber* echo_subscriber = Domain::createSubscriber(mp_participant,echo_read_param,(SubscriberListener*)&echoReplyListener);

    if(echo_subscriber == nullptr)
        return 1;





    /* For printing progress in a single line with dots */
    setbuf(stdout, NULL);

    sleep(100);

    bool success = runTests("dds", &sendMessage, NULL, mainString);

    // bye bye

    Domain::removeParticipant(mp_participant);
    return success == true;
}
