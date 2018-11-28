//========================================================
/**
 *  @file sample_pub.cpp
 */
//========================================================

#include "AckDataSender.h"
#include "EchoDataSender.h"
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

#include "test_wrapper.h"

DDSPerfTest::PubMessage pubmessage_data;

AckDataSenderImpl *ack_listener_servant;
EchoDataSenderImpl *echo_listener_servant;

bool sendMessage(void *context, void *payloadData, size_t payloadSize, long long int *elapsedTime, bool isEcho) {

    if (payloadSize != pubmessage_data.buffer.length()) {

        // maximum size we are using
        pubmessage_data.buffer.length(payloadSize);
        for (unsigned int i = 0; i < payloadSize; i++)
            pubmessage_data.buffer[i] = '*';
    }



    bool success = false;
    do {


        long count = 0;
        long sleepUsec = 10;
        long countTimeout = 10000000/sleepUsec;

        struct timespec time_start;
        TIME_MEASURE_START(time_start);
        if (isEcho) {
            echo_listener_servant->sendMessage(pubmessage_data);
            while (!echo_listener_servant->isEchoReceived() && count < countTimeout) {
                usleep(sleepUsec);
                count ++;
            }
            success = echo_listener_servant->isEchoReceived();
        } else {
            ack_listener_servant->sendMessage(pubmessage_data);
            while (!ack_listener_servant->isAckReceived() && count < countTimeout) {
                usleep(sleepUsec);
                count ++;
            }
            success = ack_listener_servant->isAckReceived();
        }
        TIME_MEASURE_DIFF_USEC(time_start, *elapsedTime);
        if (!success)
            printf("Timeout. Resending ...");
    } while (!success);

    return true;
}

int ACE_TMAIN(int argc, ACE_TCHAR *argv[]) {


    ReliabilityQosPolicyKind  reliabilityQosPolicyKind = DDS::BEST_EFFORT_RELIABILITY_QOS;
    
    try {

        // Calling TheParticipantFactoryWithArgs before user application parse command
        // line.
        DDS::DomainParticipantFactory_var dpf =
                TheParticipantFactoryWithArgs (argc, argv);

        //bool useTCP = true;
        bool useZeroCopyRead = true;
        DomainId_t myDomain = 111;

        std::setbuf(stdout, NULL); /* no buffering for standard-out */

        /* Try to set realtime scheduling class*/
        set_rt();

        /* Create participant */
        DDS::DomainParticipant_var dp =
                dpf->create_participant(myDomain,
                                        PARTICIPANT_QOS_DEFAULT,
                                        DDS::DomainParticipantListener::_nil(),
                                        ::OpenDDS::DCPS::DEFAULT_STATUS_MASK);
        if (CORBA::is_nil(dp.in())) {
            std::cout << argv[0] << " SAMPLE_PUB: ERROR - Create participant failed." << endl;
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

        /* Create topic for datawriter */
        PubMessageTypeSupportImpl *pubmessage_dt = new PubMessageTypeSupportImpl;
        DDS::ReturnCode_t register_status = pubmessage_dt->register_type(dp.in(),
                                                                         "DDSPerfTest::PubMessage");
        if (register_status != DDS::RETCODE_OK) {
            std::cerr << "ERROR: sample_pub failed to register PubMessage with return code "
                      << register_status << std::endl;
            exit(1);
        }

        DDS::Topic_var ack_send_topic = dp->create_topic(TOPIC_ACK_SEND, // topic name
                                                         "DDSPerfTest::PubMessage", // topic type
                                                         topicQos,
                                                         DDS::TopicListener::_nil(),
                                                         ::OpenDDS::DCPS::DEFAULT_STATUS_MASK);

        DDS::DataWriterQos ackWriterQos;
        ack_pub->get_default_datawriter_qos(ackWriterQos);
        ackWriterQos.resource_limits.max_instances = 1;
        ackWriterQos.resource_limits.max_samples = 1;
        ackWriterQos.reliability.kind = reliabilityQosPolicyKind;


        /* Create PubMessage datawriter */
        DDS::DataWriter_var ack_dw = ack_pub->create_datawriter(ack_send_topic.in(),
                                                                ackWriterQos,
                                                                DDS::DataWriterListener::_nil(),
                                                                ::OpenDDS::DCPS::DEFAULT_STATUS_MASK);
        PubMessageDataWriter_var ack_writer =
                PubMessageDataWriter::_narrow(ack_dw.in());


        /**
         * ACK Subscriber
         */

        /* Create the subscriber */
        DDS::Subscriber_var ack_sub =
                dp->create_subscriber(SUBSCRIBER_QOS_DEFAULT,
                                      DDS::SubscriberListener::_nil(),
                                      ::OpenDDS::DCPS::DEFAULT_STATUS_MASK);

        /* Create topic for datareader */
        AckMessageTypeSupportImpl *ackmessage_dt = new AckMessageTypeSupportImpl;
        register_status = ackmessage_dt->register_type(dp.in(),
                                                       "DDSPerfTest::AckMessage");
        if (register_status != DDS::RETCODE_OK) {
            std::cerr << "ERROR: sample_pub failed to register AckMessage with return code "
                      << register_status << std::endl;
            exit(1);
        }

        DDS::Topic_var ack_rec_topic = dp->create_topic(TOPIC_ACK_REC, // topic name
                                                        "DDSPerfTest::AckMessage", // topic type
                                                        topicQos,
                                                        DDS::TopicListener::_nil(),
                                                        ::OpenDDS::DCPS::DEFAULT_STATUS_MASK);

        /* Create the listener for datareader */
        DDS::DataReaderListener_var ack_listener(new AckDataSenderImpl());
        ack_listener_servant =
                dynamic_cast<AckDataSenderImpl *>(ack_listener.in());


        DDS::DataReaderQos ackReaderQos;
        ack_sub->get_default_datareader_qos(ackReaderQos);
        ackReaderQos.resource_limits.max_instances = 1;
        ackReaderQos.resource_limits.max_samples = 1;
        ackReaderQos.reliability.kind = reliabilityQosPolicyKind;

        /* Create AckMessage datareader */
        DDS::DataReader_var ack_dr = ack_sub->create_datareader(ack_rec_topic.in(),
                                                                ackReaderQos,
                                                                ack_listener.in(),
                                                                ::OpenDDS::DCPS::DEFAULT_STATUS_MASK);

        ack_listener_servant->init(ack_dr.in(), ack_dw.in(), useZeroCopyRead);

        /**
         * ECHO Publisher
         */

        /* Create publisher */
        DDS::Publisher_var echo_pub =
                dp->create_publisher(PUBLISHER_QOS_DEFAULT,
                                     DDS::PublisherListener::_nil(),
                                     ::OpenDDS::DCPS::DEFAULT_STATUS_MASK);

        /* Create topic for datawriter */
        DDS::Topic_var echo_send_topic = dp->create_topic(TOPIC_ECHO_SEND, // topic name
                                                         "DDSPerfTest::PubMessage", // topic type
                                                         topicQos,
                                                         DDS::TopicListener::_nil(),
                                                         ::OpenDDS::DCPS::DEFAULT_STATUS_MASK);


        DDS::DataWriterQos echoWriterQos;
        echo_pub->get_default_datawriter_qos(echoWriterQos);
        echoWriterQos.resource_limits.max_instances = 1;
        echoWriterQos.resource_limits.max_samples = 1;
        echoWriterQos.reliability.kind = reliabilityQosPolicyKind;

        /* Create PubMessage datawriter */
        DDS::DataWriter_var echo_dw = echo_pub->create_datawriter(echo_send_topic.in(),
                                                                  echoWriterQos,
                                                                DDS::DataWriterListener::_nil(),
                                                                ::OpenDDS::DCPS::DEFAULT_STATUS_MASK);
        PubMessageDataWriter_var echo_writer =
                PubMessageDataWriter::_narrow(echo_dw.in());


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
        register_status = echomessage_dt->register_type(dp.in(),
                                                       "DDSPerfTest::PubMessage");
        if (register_status != DDS::RETCODE_OK) {
            std::cerr << "ERROR: sample_pub failed to register PubMessage with return code "
                      << register_status << std::endl;
            exit(1);
        }

        DDS::Topic_var echo_rec_topic = dp->create_topic(TOPIC_ECHO_REC, // topic name
                                                        "DDSPerfTest::PubMessage", // topic type
                                                        topicQos,
                                                        DDS::TopicListener::_nil(),
                                                        ::OpenDDS::DCPS::DEFAULT_STATUS_MASK);

        /* Create the listener for datareader */
        DDS::DataReaderListener_var echo_listener(new EchoDataSenderImpl());
        echo_listener_servant =
                dynamic_cast<EchoDataSenderImpl *>(echo_listener.in());


        DDS::DataReaderQos echoReaderQos;
        echo_sub->get_default_datareader_qos(echoReaderQos);
        echoReaderQos.resource_limits.max_instances = 1;
        echoReaderQos.resource_limits.max_samples = 1;
        echoReaderQos.reliability.kind = reliabilityQosPolicyKind;

        /* Create PubMessage datareader */
        DDS::DataReader_var echo_dr = echo_sub->create_datareader(echo_rec_topic.in(),
                                                                  echoReaderQos,
                                                                echo_listener.in(),
                                                                ::OpenDDS::DCPS::DEFAULT_STATUS_MASK);

        echo_listener_servant->init(echo_dr.in(), echo_dw.in(), useZeroCopyRead);


        /**
         * Run tests
         */



        bool success = runTests("dds", &sendMessage, NULL, NULL);

        /* Shut down domain entities */

        dp->delete_contained_entities();
        dpf->delete_participant(dp.in());
        TheServiceParticipant->shutdown();

    } catch (const OpenDDS::DCPS::Transport::MiscProblem &) {
        ACE_ERROR((LM_ERROR,
                ACE_TEXT("(%P|%t) sample_pub() - ")
                          ACE_TEXT("Transport::MiscProblem exception caught during processing.\n")
                  ));
        return 1;
    } catch (const OpenDDS::DCPS::Transport::NotFound &) {
        ACE_ERROR((LM_ERROR,
                ACE_TEXT("(%P|%t) sample_pub() - ")
                          ACE_TEXT("Transport::NotFound exception caught during processing.\n")
                  ));
        return 1;
    }

    return 0;

}
