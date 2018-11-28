// -*- C++ -*-
//
#include "EchoDataReply.h"
#include "DDSPerfTestTypeSupportImpl.h"
#include "DDSPerfTestTypeSupportC.h"
#include <dds/DCPS/Service_Participant.h>
#include <ace/streams.h>
#include <time_utils.h>

using namespace DDSPerfTest;

EchoDataReplyListenerImpl::EchoDataReplyListenerImpl()
        : writer_ (),
          reader_ (),
          dr_servant_(),
          dw_servant_(),
          handle_(),
          use_zero_copy_(false)
{
}

void EchoDataReplyListenerImpl::init(DDS::DataReader_ptr dr,
                                     DDS::DataWriter_ptr dw,
                                     bool use_zero_copy_read, std::string _readTopic,
                                     std::string _writeTopic)
{
    this->writer_ = DDS::DataWriter::_duplicate (dw);
    this->reader_ = DDS::DataReader::_duplicate (dr);
    use_zero_copy_ = use_zero_copy_read;
    this->readTopic = _readTopic;
    this->writeTopic = _writeTopic;

    this->dw_servant_ =
            PubMessageDataWriter::_narrow (this->writer_.in ());
    DDSPerfTest::PubMessage msg;
    this->handle_ = this->dw_servant_->register_instance(msg);

    this->dr_servant_ =
            PubMessageDataReader::_unchecked_narrow(this->reader_.in());
}

EchoDataReplyListenerImpl::~EchoDataReplyListenerImpl ()
{
}

void EchoDataReplyListenerImpl::on_data_available(DDS::DataReader_ptr)
{
    if (use_zero_copy_)
    {
        ::CORBA::Long max_read_samples = 1;
        DDSPerfTest::PubMessageSeq message;
        DDS::SampleInfoSeq              si;
        // Use the reader data member (instead of the argument) for efficiency
        // reasons

        //long elapsedTime, elapsedTime2, elapsedTime3;
        //struct timespec time_start1;
        //TIME_MEASURE_START(time_start1);
        DDS::ReturnCode_t status = this->dr_servant_->take(message, si, max_read_samples,
                                ::DDS::NOT_READ_SAMPLE_STATE,
                                ::DDS::ANY_VIEW_STATE,
                                ::DDS::ANY_INSTANCE_STATE);
        //TIME_MEASURE_DIFF_USEC(time_start1, elapsedTime);


        if (status != DDS::RETCODE_OK) {
            std::cerr << "ERROR: Can not take message from ECHO receive" << std::endl;
            exit(1);
        }


        //cout << "Sending echo for " << message[0].buffer.length() << endl;
        //struct timespec time_start;
        //TIME_MEASURE_START(time_start);
        status = this->dw_servant_->write (message[0], this->handle_);
        //TIME_MEASURE_DIFF_USEC(time_start, elapsedTime2);

        //cout << "Forward echo: " << readTopic << " -> " << writeTopic << endl;

        if (status != DDS::RETCODE_OK) {
            std::cerr << "ERROR: Can not write message" << std::endl;
            exit(1);
        }

        status = this->dr_servant_->return_loan(message, si);

        if (status != DDS::RETCODE_OK) {
            std::cerr << "ERROR: Can not return message loan" << std::endl;
            exit(1);
        }


        //TIME_MEASURE_DIFF_USEC(time_start1, elapsedTime2);
        //std::cout << "Take " << elapsedTime << " Write " << elapsedTime2 << " Total " << elapsedTime3 << endl;
    }
    else
    {
        DDSPerfTest::PubMessage message;
        DDS::SampleInfo si;
        // Use the reader data member (instead of the argument) for efficiency
        // reasons
        this->dr_servant_->take_next_sample(message, si) ;


        //cout << "Sending echo for " << message.buffer.length() << endl;
        DDS::ReturnCode_t status = this->dw_servant_->write (message, this->handle_);


        if (status != DDS::RETCODE_OK) {
            std::cerr << "ERROR: Can not write message" << std::endl;
            exit(1);
        }

    }

}

void EchoDataReplyListenerImpl::on_requested_deadline_missed (
        DDS::DataReader_ptr,
        const DDS::RequestedDeadlineMissedStatus &)
{
}

void EchoDataReplyListenerImpl::on_requested_incompatible_qos (
        DDS::DataReader_ptr,
        const DDS::RequestedIncompatibleQosStatus &)
{
}

void EchoDataReplyListenerImpl::on_liveliness_changed (
        DDS::DataReader_ptr,
        const DDS::LivelinessChangedStatus &)
{
}

void EchoDataReplyListenerImpl::on_subscription_matched (
        DDS::DataReader_ptr,
        const DDS::SubscriptionMatchedStatus &)
{
}

void EchoDataReplyListenerImpl::on_sample_rejected(
        DDS::DataReader_ptr,
        const DDS::SampleRejectedStatus&)
{
}

void EchoDataReplyListenerImpl::on_sample_lost(
        DDS::DataReader_ptr,
        const DDS::SampleLostStatus&)
{
}
