// -*- C++ -*-
//
#include "AckDataReply.h"
#include "DDSPerfTestTypeSupportImpl.h"
#include "DDSPerfTestTypeSupportC.h"
#include <dds/DCPS/Service_Participant.h>
#include <ace/streams.h>

using namespace DDSPerfTest;

AckDataReplyListenerImpl::AckDataReplyListenerImpl()
        : writer_ (),
          reader_ (),
          dr_servant_(),
          dw_servant_(),
          handle_(),
          use_zero_copy_(false)
{
}

void AckDataReplyListenerImpl::init(DDS::DataReader_ptr dr,
                                     DDS::DataWriter_ptr dw,
                                     bool use_zero_copy_read)
{
    this->writer_ = DDS::DataWriter::_duplicate (dw);
    this->reader_ = DDS::DataReader::_duplicate (dr);
    use_zero_copy_ = use_zero_copy_read;

    this->dw_servant_ =
            AckMessageDataWriter::_narrow (this->writer_.in ());
    DDSPerfTest::AckMessage msg;
    this->handle_ = this->dw_servant_->register_instance(msg);

    this->dr_servant_ =
            PubMessageDataReader::_unchecked_narrow(this->reader_.in());
}

AckDataReplyListenerImpl::~AckDataReplyListenerImpl ()
{
}

void AckDataReplyListenerImpl::on_data_available(DDS::DataReader_ptr)
{
    CORBA::Long rec_size;

    if (use_zero_copy_)
    {
        ::CORBA::Long max_read_samples = 1;
        DDSPerfTest::PubMessageSeq message;
        DDS::SampleInfoSeq              si;
        // Use the reader data member (instead of the argument) for efficiency
        // reasons

        DDS::ReturnCode_t status = this->dr_servant_->take(message, si, max_read_samples,
                                ::DDS::NOT_READ_SAMPLE_STATE,
                                ::DDS::ANY_VIEW_STATE,
                                ::DDS::ANY_INSTANCE_STATE);

        if (status != DDS::RETCODE_OK) {
            std::cerr << "ERROR: Can not take message from ACK receive" << std::endl;
            exit(1);
        }


        rec_size = message[0].buffer.length();
        status = this->dr_servant_->return_loan(message, si);

        if (status != DDS::RETCODE_OK) {
            std::cerr << "ERROR: Can not return message loan" << std::endl;
            exit(1);
        }
    }
    else
    {
        DDSPerfTest::PubMessage message;
        DDS::SampleInfo si;
        // Use the reader data member (instead of the argument) for efficiency
        // reasons
        this->dr_servant_->take_next_sample(message, si) ;

        rec_size = message.buffer.length();
    }

    //cout << "Sending ack for " << rec_size << endl;

    DDSPerfTest::AckMessage msg;
    msg.size = rec_size;

    this->dw_servant_->write (msg, this->handle_);

}

void AckDataReplyListenerImpl::on_requested_deadline_missed (
        DDS::DataReader_ptr,
        const DDS::RequestedDeadlineMissedStatus &)
{
}

void AckDataReplyListenerImpl::on_requested_incompatible_qos (
        DDS::DataReader_ptr,
        const DDS::RequestedIncompatibleQosStatus &)
{
}

void AckDataReplyListenerImpl::on_liveliness_changed (
        DDS::DataReader_ptr,
        const DDS::LivelinessChangedStatus &)
{
}

void AckDataReplyListenerImpl::on_subscription_matched (
        DDS::DataReader_ptr,
        const DDS::SubscriptionMatchedStatus &)
{
}

void AckDataReplyListenerImpl::on_sample_rejected(
        DDS::DataReader_ptr,
        const DDS::SampleRejectedStatus&)
{
}

void AckDataReplyListenerImpl::on_sample_lost(
        DDS::DataReader_ptr,
        const DDS::SampleLostStatus&)
{
}
