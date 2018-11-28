//========================================================
/**
 *  @file sample_pub.cpp
 */
//========================================================
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

#include "proc_stat.h"

#include "common.h"

using namespace std;

using namespace eprosima::fastrtps;
using namespace eprosima::fastrtps::rtps;


bool running = true;

static void stopHandler(int sig) {
    running = false;
}

int receivedCount = 0;


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
                receivedCount++;
                std::cout << "Got message with size: " << pubMessage.buffer().size() << std::endl;
            }
        }

    }
    DDSPerfTest::PubMessage pubMessage;
    eprosima::fastrtps::SampleInfo_t m_info;
};

int main(int argc, char** argv)
{

    signal(SIGINT, stopHandler);
    signal(SIGTERM, stopHandler);
    signal(SIGUSR1, stopHandler);

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
    PParam.rtps.setName("Participant_server");
    mp_participant = Domain::createParticipant(PParam);

    if(mp_participant==nullptr)
        return 1;
    
    //REGISTER THE TYPE
    DDSPerfTest::AckMessagePubSubType m_type_ack;
    DDSPerfTest::PubMessagePubSubType m_type_pub;
    Domain::registerType(mp_participant,&m_type_ack);
    Domain::registerType(mp_participant,&m_type_pub);

    //CREATE THE SUBSCRIBER
    SubscriberAttributes ack_read_param;
    ack_read_param.topic.topicKind = NO_KEY;
    ack_read_param.topic.topicDataType = "DDSPerfTest::PubMessage";
    ack_read_param.topic.topicName = TOPIC_ACK_SEND;
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

    // we expect 10 messages from the client, then we shut down.
    int expectedCount = 5;

    printf("Overhead server ready to receive DDS messages\n");
    while(running && receivedCount < expectedCount)
        usleep(1000);

    Domain::removeParticipant(mp_participant);

    return (0);

}
