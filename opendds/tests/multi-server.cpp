//
// Created by profanter on 9/1/18.
// Copyright (c) 2018 fortiss GmbH. All rights reserved.
//

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


#include "common.h"

using namespace DDS;
using namespace CORBA;
using namespace DDSPerfTest;

bool running = true;
static void stopHandler(int sig) {
    running = false;
}

int ACE_TMAIN (int argc, ACE_TCHAR *argv[]) {


    signal(SIGINT, stopHandler);
    signal(SIGTERM, stopHandler);

    std::string topicSelf;
    std::string topicForward;

    try
    {
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
        if (CORBA::is_nil (dp.in ())) {
            cerr << "create_participant failed." << endl;
            return 1;
        }


        int useRtpsSub = 0;
        int useRtpsPub = 0;

        ACE_Get_Opt get_opts (argc, argv, ACE_TEXT("s:f:x:y:"));
        int ich;
        while ((ich = get_opts ()) != EOF) {
            switch (ich) {

                case 's': /* self topic */
                    topicSelf = std::string(get_opts.opt_arg());
                    break;
                case 'f': /* forward topic */
                    topicForward = std::string(get_opts.opt_arg());
                    break;

                case 'x': // forward topic
                    useRtpsPub = ACE_OS::atoi(get_opts.opt_arg());

                    break;

                case 'y': // forward topic
                    useRtpsSub = ACE_OS::atoi(get_opts.opt_arg());

                    break;

                default: /* no parameters */
                    break;

            }
        }

        if (topicSelf.length() == 0 || topicForward.length() == 0) {
            cerr << "Specify the selfTopic (command option -s) and forwardTopic (command option -f)" << endl;
            exit(1);
        }

        /*if (useRtpsPub)
            cout << " -- Using RTPS publish transport: " << topicForward << endl;
        else
            cout << " -- Using SHM publish transport: " << topicForward << endl;
        if (useRtpsSub)
            cout << " -- Using RTPS subscribe transport: " << topicSelf << endl;
        else
            cout << " -- Using SHM scubscribe transport: " << topicSelf << endl;*/


        DDS::TopicQos topicQos;
        dp->get_default_topic_qos(topicQos);
        topicQos.resource_limits.max_instances = 1;
        topicQos.resource_limits.max_samples = 1;
        topicQos.reliability.kind = DDS::BEST_EFFORT_RELIABILITY_QOS;


        /**
         * ECHO Publisher
         */

        /* Create publisher */
        DDS::Publisher_var echo_pub =
                dp->create_publisher(PUBLISHER_QOS_DEFAULT,
                                     DDS::PublisherListener::_nil(),
                                     ::OpenDDS::DCPS::DEFAULT_STATUS_MASK);

        OpenDDS::DCPS::TransportConfig_rch pub_transport =
                TheTransportRegistry->create_config("t1");
        if (useRtpsPub) {
            pub_transport->instances_.push_back(
                    TheTransportRegistry->create_inst("t1_rtps_udp", "rtps_udp"));
        } else {
            pub_transport->instances_.push_back(
                    TheTransportRegistry->create_inst("t1_shmem", "shmem"));
        }

        // Attach the transport protocol with the publishing entity
        TheTransportRegistry->bind_config(pub_transport, echo_pub);

        DDS::DataWriterQos echoWriterQos;
        echo_pub->get_default_datawriter_qos(echoWriterQos);
        echoWriterQos.resource_limits.max_instances = 1;
        echoWriterQos.resource_limits.max_samples = 1;
        echoWriterQos.reliability.kind = DDS::BEST_EFFORT_RELIABILITY_QOS;

        /* Create topic for datawriter */
        PubMessageTypeSupportImpl *pubMessage_dt = new PubMessageTypeSupportImpl;
        pubMessage_dt->register_type(dp.in(),
                                     "DDSPerfTest::PubMessage");
        DDS::Topic_var echomessage_topic = dp->create_topic(topicForward.c_str(), // topic name
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

        OpenDDS::DCPS::TransportConfig_rch sub_transport =
                TheTransportRegistry->create_config("t2");
        if (useRtpsSub) {
            sub_transport->instances_.push_back(
                    TheTransportRegistry->create_inst("t2_rtps_udp", "rtps_udp"));
        } else {
            sub_transport->instances_.push_back(
                    TheTransportRegistry->create_inst("t2_shmem", "shmem"));
        }

        // Attach the transport protocol with the publishing entity
        TheTransportRegistry->bind_config(sub_transport, echo_sub);


        DDS::DataReaderQos echoReaderQos;
        echo_sub->get_default_datareader_qos(echoReaderQos);
        echoReaderQos.resource_limits.max_instances = 1;
        echoReaderQos.resource_limits.max_samples = 1;
        echoReaderQos.reliability.kind = DDS::BEST_EFFORT_RELIABILITY_QOS;

        /* Create topic for datareader */
        DDS::Topic_var echo_send_topic = dp->create_topic(topicSelf.c_str(), // topic name
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

        echo_reply_listener_servant->init(echo_send_dr.in(), echo_rec_dw.in(), true, topicSelf, topicForward);

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
    }
    catch (CORBA::Exception& e)
    {
        cerr << "PUB: Exception caught in main.cpp:" << endl
             << e << endl;
        exit(1);
    }

    return 0;
}
