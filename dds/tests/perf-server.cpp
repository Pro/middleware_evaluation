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

eprosima::fastrtps::Publisher* ack_publisher;
eprosima::fastrtps::Publisher* echo_publisher;


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
                ackMessage.size((int)pubMessage.buffer().size());

                if (!ack_publisher->write((void*)&ackMessage)) {
                    std::cerr << "Could not write ack reply" << std::endl;
                }
            } else {
                //std::cout << "Sent ack reply" << std::endl;
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

int main(int argc, char** argv)
{

    signal(SIGINT, stopHandler);
    signal(SIGTERM, stopHandler);
    signal(SIGUSR1, stopHandler);

    char procStatFile[255];
    struct tm *timenow;
    time_t now = time(NULL);
    timenow = gmtime(&now);
    strftime(procStatFile, 255, "procStat-dds-%Y-%m-%d_%H%M%S.csv",timenow);

    setProcPriority();

    int procStatPid = runProcStatToCsv(procStatFile);
    if (procStatPid <= 0)
        return 1;


    printf("Process Status writing to: %s\n", procStatFile);

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

    //CREATE THE ACK PUBLISHER
    PublisherAttributes ack_write_param;
    ack_write_param.topic.topicKind = NO_KEY;
    ack_write_param.topic.topicDataType = "DDSPerfTest::AckMessage";
    ack_write_param.topic.topicName = TOPIC_ACK_REC;
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
    
    //CREATE THE ECHO PUBLISHER
    PublisherAttributes echo_write_param;
    echo_write_param.topic.topicKind = NO_KEY;
    echo_write_param.topic.topicDataType = "DDSPerfTest::PubMessage";
    echo_write_param.topic.topicName = TOPIC_ECHO_REC;
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
    echo_read_param.topic.topicName = TOPIC_ECHO_SEND;
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


    printf("Echo server ready to receive DDS messages\n");
    while(running)
        usleep(1000);

    Domain::removeParticipant(mp_participant);


    stopProcToCsv(procStatPid);

    printf("Process Status written to: %s\n", procStatFile);

    return (0);

}
