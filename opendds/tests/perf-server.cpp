//========================================================
/**
 *  @file sample_pub.cpp
 */
//========================================================

#include "AckDataReply.h"
#include "EchoDataReply.h"
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

#include <cstdio>

#include "proc_stat.h"

#include "common.h"

using namespace DDS;
using namespace CORBA;
using namespace DDSPerfTest;
using namespace std;


bool running = true;

static void stopHandler(int sig) {
    running = false;
}



int ACE_TMAIN(int argc, ACE_TCHAR *argv[]) {

    signal(SIGINT, stopHandler);
    signal(SIGTERM, stopHandler);
    signal(SIGUSR1, stopHandler);

    char procStatFile[255];
    struct tm *timenow;
    time_t now = time(NULL);
    timenow = gmtime(&now);
    strftime(procStatFile, 255, "procStat-dds-%Y-%m-%d_%H%M%S.csv",timenow);


    ReliabilityQosPolicyKind  reliabilityQosPolicyKind = DDS::BEST_EFFORT_RELIABILITY_QOS;

    setProcPriority();

    printf("Process Status writing to: %s\n", procStatFile);

    int procStatPid = runProcStatToCsv(procStatFile);
    if (procStatPid <= 0)
        return 1;


    try {

        // Calling TheParticipantFactoryWithArgs before user application parse command
        // line.
        /* Create participant */
        DDS::DomainParticipantFactory_var dpf =
                TheParticipantFactoryWithArgs (argc, argv);

        bool useZeroCopyRead = true;
        DomainId_t myDomain = 111;

        std::setbuf(stdout, NULL);

        /* Try to set realtime scheduling class*/
        set_rt();


        /* Create participant */
        DDS::DomainParticipant_var dp =
                dpf->create_participant(myDomain,
                                        PARTICIPANT_QOS_DEFAULT,
                                        DDS::DomainParticipantListener::_nil(),
                                        ::OpenDDS::DCPS::DEFAULT_STATUS_MASK);
        if (CORBA::is_nil(dp.in())) {
            cout << argv[0] << "SAMPLE_SUB: ERROR - Create participant failed." << endl;
            stopProcToCsv(procStatPid);
            exit(1);
        }


        DDS::TopicQos topicQos;
        dp->get_default_topic_qos(topicQos);
        topicQos.resource_limits.max_instances = 1;
        topicQos.resource_limits.max_samples = 1;
        topicQos.reliability.kind = reliabilityQosPolicyKind;

        /**
         * ACK Publisher
         */

        /* Create publisher */
        DDS::Publisher_var ack_pub =
                dp->create_publisher(PUBLISHER_QOS_DEFAULT,
                                     DDS::PublisherListener::_nil(),
                                     ::OpenDDS::DCPS::DEFAULT_STATUS_MASK);


        DDS::DataWriterQos ackWriterQos;
        ack_pub->get_default_datawriter_qos(ackWriterQos);
        ackWriterQos.resource_limits.max_instances = 1;
        ackWriterQos.resource_limits.max_samples = 1;
        ackWriterQos.reliability.kind = reliabilityQosPolicyKind;

        /* Create topic for datawriter */
        AckMessageTypeSupportImpl *ackmessage_dt = new AckMessageTypeSupportImpl;
        ackmessage_dt->register_type(dp.in(),
                                     "DDSPerfTest::AckMessage");
        DDS::Topic_var ackmessage_topic = dp->create_topic(TOPIC_ACK_REC, // topic name
                                                           "DDSPerfTest::AckMessage", // topic type
                                                           topicQos,
                                                           DDS::TopicListener::_nil(),
                                                           ::OpenDDS::DCPS::DEFAULT_STATUS_MASK);

        /* Create PubMessage data writer */
        DDS::DataWriter_var ack_rec_dw = ack_pub->create_datawriter(ackmessage_topic.in(),
                                                                    ackWriterQos,
                                                      DDS::DataWriterListener::_nil(),
                                                      ::OpenDDS::DCPS::DEFAULT_STATUS_MASK);
        AckMessageDataWriter_var ackmessage_writer =
                AckMessageDataWriter::_narrow(ack_rec_dw.in());


        /**
         * ACK Subscriber
         */

        /* Create the subscriber */
        DDS::Subscriber_var ack_sub =
                dp->create_subscriber(SUBSCRIBER_QOS_DEFAULT,
                                      DDS::SubscriberListener::_nil(),
                                      ::OpenDDS::DCPS::DEFAULT_STATUS_MASK);


        DDS::DataReaderQos ackReaderQos;
        ack_sub->get_default_datareader_qos(ackReaderQos);
        ackReaderQos.resource_limits.max_instances = 1;
        ackReaderQos.resource_limits.max_samples = 1;
        ackReaderQos.reliability.kind = reliabilityQosPolicyKind;

        /* Create topic for datareader */
        PubMessageTypeSupportImpl *pubmessage_dt = new PubMessageTypeSupportImpl;
        pubmessage_dt->register_type(dp.in(),
                                     "DDSPerfTest::PubMessage");
        DDS::Topic_var ack_send_topic = dp->create_topic(TOPIC_ACK_SEND, // topic name
                                                           "DDSPerfTest::PubMessage", // topic type
                                                           topicQos,
                                                           DDS::TopicListener::_nil(),
                                                           ::OpenDDS::DCPS::DEFAULT_STATUS_MASK);

        /* Create the ack_listener for datareader */
        DDS::DataReaderListener_var ack_listener(new AckDataReplyListenerImpl);
        AckDataReplyListenerImpl *ack_reply_listener_servant =
                dynamic_cast<AckDataReplyListenerImpl *>(ack_listener.in());

        /* Create AckMessage datareader */
        DDS::DataReader_var ack_send_dr = ack_sub->create_datareader(ack_send_topic.in(),
                                                                     ackReaderQos,
                                                      ack_listener.in(),
                                                      ::OpenDDS::DCPS::DEFAULT_STATUS_MASK);

        ack_reply_listener_servant->init(ack_send_dr.in(), ack_rec_dw.in(), useZeroCopyRead);

        /**
         * ECHO Publisher
         */

        /* Create publisher */
        DDS::Publisher_var echo_pub =
                dp->create_publisher(PUBLISHER_QOS_DEFAULT,
                                     DDS::PublisherListener::_nil(),
                                     ::OpenDDS::DCPS::DEFAULT_STATUS_MASK);


        DDS::DataWriterQos echoWriterQos;
        echo_pub->get_default_datawriter_qos(echoWriterQos);
        echoWriterQos.resource_limits.max_instances = 1;
        echoWriterQos.resource_limits.max_samples = 1;
        echoWriterQos.reliability.kind = reliabilityQosPolicyKind;

        /* Create topic for datawriter */
        PubMessageTypeSupportImpl *pubMessage_dt = new PubMessageTypeSupportImpl;
        pubMessage_dt->register_type(dp.in(),
                                     "DDSPerfTest::PubMessage");
        DDS::Topic_var echomessage_topic = dp->create_topic(TOPIC_ECHO_REC, // topic name
                                                           "DDSPerfTest::PubMessage", // topic type
                                                           topicQos,
                                                           DDS::TopicListener::_nil(),
                                                           ::OpenDDS::DCPS::DEFAULT_STATUS_MASK);

        /* Create PubMessage data writer */
        DDS::DataWriter_var echo_rec_dw = echo_pub->create_datawriter(echomessage_topic.in(),
                                                                      echoWriterQos,
                                                      DDS::DataWriterListener::_nil(),
                                                      ::OpenDDS::DCPS::DEFAULT_STATUS_MASK);
        PubMessageDataWriter_var echomessage_writer =
                PubMessageDataWriter::_narrow(echo_rec_dw.in());


        /**
         * ECHO Subscriber
         */

        /* Create the subscriber */
        DDS::Subscriber_var echo_sub =
                dp->create_subscriber(SUBSCRIBER_QOS_DEFAULT,
                                      DDS::SubscriberListener::_nil(),
                                      ::OpenDDS::DCPS::DEFAULT_STATUS_MASK);

        DDS::DataReaderQos echoReaderQos;
        echo_sub->get_default_datareader_qos(echoReaderQos);
        echoReaderQos.resource_limits.max_instances = 1;
        echoReaderQos.resource_limits.max_samples = 1;
        echoReaderQos.reliability.kind = reliabilityQosPolicyKind;

        /* Create topic for datareader */
        DDS::Topic_var echo_send_topic = dp->create_topic(TOPIC_ECHO_SEND, // topic name
                                                           "DDSPerfTest::PubMessage", // topic type
                                                           topicQos,
                                                           DDS::TopicListener::_nil(),
                                                           ::OpenDDS::DCPS::DEFAULT_STATUS_MASK);

        /* Create the echo_listener for datareader */
        DDS::DataReaderListener_var echo_listener(new EchoDataReplyListenerImpl);
        EchoDataReplyListenerImpl *echo_reply_listener_servant =
                dynamic_cast<EchoDataReplyListenerImpl *>(echo_listener.in());

        /* Create PubMessage datareader */
        DDS::DataReader_var echo_send_dr = echo_sub->create_datareader(echo_send_topic.in(),
                                                                       echoReaderQos,
                                                      echo_listener.in(),
                                                      ::OpenDDS::DCPS::DEFAULT_STATUS_MASK);

        echo_reply_listener_servant->init(echo_send_dr.in(), echo_rec_dw.in(), useZeroCopyRead);

        while (running) {
            if (usleep(1000) != 0)
                break;
        }



        std::cout << "Sub: shut down" << std::endl;
        /* Shut down domain entities */
        echo_pub->delete_contained_entities();
        dp->delete_contained_entities();
        dpf->delete_participant(dp.in());
        TheServiceParticipant->shutdown();

    } catch (const OpenDDS::DCPS::Transport::MiscProblem &) {
        stopProcToCsv(procStatPid);
        ACE_ERROR((LM_ERROR,
                ACE_TEXT("(%P|%t) sample_sub() - ")
                          ACE_TEXT("Transport::MiscProblem exception caught during processing.\n")
                  ));
        return 1;
    } catch (const OpenDDS::DCPS::Transport::NotFound &) {
        stopProcToCsv(procStatPid);
        ACE_ERROR((LM_ERROR,
                ACE_TEXT("(%P|%t) sample_sub() - ")
                          ACE_TEXT("Transport::NotFound exception caught during processing.\n")
                  ));
        return 1;
    }
    stopProcToCsv(procStatPid);

    printf("Process Status written to: %s\n", procStatFile);

    return (0);

}
