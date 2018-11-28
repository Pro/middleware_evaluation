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


int main(int argc, char *argv[]) {


    if (argc != 2) {
        printf("Usage: program <PAYLOAD_BYTES>\n");
        return 1;
    }


    int payloadSize = atoi(argv[1]);

    if (payloadSize < 0){
        printf("Payload size cannot be negative\n");
        return 1;
    }

    eprosima::fastrtps::Participant* mp_participant;

    DDSPerfTest::PubMessage pubmessage_data = DDSPerfTest::PubMessage();
    printf("Peparing payload of %d bytes\n", payloadSize);

    pubmessage_data.buffer().resize(payloadSize, '*');

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
    eprosima::fastrtps::Publisher* ack_publisher = Domain::createPublisher(mp_participant,ack_write_param);
    if(ack_publisher == nullptr)
        return 1;

    /* For printing progress in a single line with dots */
    setbuf(stdout, NULL);


    printf("Ready. Sleeping a bit...\n");
    sleep(5);


    for (unsigned short i=0; i<5; i++) {
        printf("Sending package %d\n", i);
        if (!ack_publisher->write((void*)&pubmessage_data)) {
            std::cerr << "Could not write ack message" << std::endl;
        }
        sleep(2);
    }

    // bye bye

    Domain::removeParticipant(mp_participant);
    return 0;
}
