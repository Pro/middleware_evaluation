// -*- C++ -*-
//
#include "AckDataSender.h"
#include "DDSPerfTestTypeSupportImpl.h"
#include "DDSPerfTestTypeSupportC.h"

#include <dds/DCPS/Service_Participant.h>
#include <ace/streams.h>
#include <ace/OS_NS_time.h>

#include <time.h>
#include <math.h>
#include <string.h>

using namespace DDSPerfTest;
using namespace std;

AckDataSenderImpl::AckDataSenderImpl()
        :writer_ (),
         reader_ (),
         dr_servant_ (),
         dw_servant_ (),
         handle_ (),
         use_zero_copy_(false)
{

}

AckDataSenderImpl::~AckDataSenderImpl ()
{
}

void AckDataSenderImpl::init(DDS::DataReader_ptr dr,
                                     DDS::DataWriter_ptr dw,
                                     bool use_zero_copy_read)
{
    this->writer_ = DDS::DataWriter::_duplicate (dw);
    this->reader_ = DDS::DataReader::_duplicate (dr);
    use_zero_copy_ = use_zero_copy_read;

    this->dr_servant_ =
            AckMessageDataReader::_narrow(this->reader_.in());

    this->dw_servant_ =
            PubMessageDataWriter::_narrow (this->writer_.in ());
    DDSPerfTest::PubMessage msg;
    this->handle_ = this->dw_servant_->register_instance(msg);
}


void AckDataSenderImpl::on_data_available(DDS::DataReader_ptr)
{
    CORBA::Long rec_size;
    DDS::ReturnCode_t status;

    if (use_zero_copy_)
    {
        ::CORBA::Long max_read_samples = 1;
        AckMessageSeq messageZC;
        DDS::SampleInfoSeq siZC;
        status = this->dr_servant_->take(messageZC,
                                         siZC,
                                         max_read_samples,
                                         ::DDS::NOT_READ_SAMPLE_STATE,
                                         ::DDS::ANY_VIEW_STATE,
                                         ::DDS::ANY_INSTANCE_STATE);

        if (status != DDS::RETCODE_OK) {
            std::cerr << "ERROR: Can not take message from ACK" << std::endl;
            exit(1);
        }

        rec_size = messageZC[0].size;
        status = this->dr_servant_->return_loan(messageZC, siZC);

        if (status != DDS::RETCODE_OK) {
            std::cerr << "ERROR: Can not return loan" << std::endl;
            exit(1);
        }
    }
    else
    {
        static DDSPerfTest::AckMessage message;
        static DDS::SampleInfo si;
        status = this->dr_servant_->take_next_sample(message, si) ;

        rec_size = message.size;
    }

    if (status == DDS::RETCODE_OK) {
//      cout << "AckMessage: seqnum    = " << message.seqnum << endl;
    } else if (status == DDS::RETCODE_NO_DATA) {
        cerr << "ERROR: reader received DDS::RETCODE_NO_DATA!" << endl;
    } else {
        cerr << "ERROR: read Message: Error: " <<  status << endl;
    }

    ackRecieved = true;

}


void AckDataSenderImpl::sendMessage(DDSPerfTest::PubMessage &pubmessage_data) {
    ackRecieved = false;
    DDS::ReturnCode_t status = this->dw_servant_->write (pubmessage_data, this->handle_);
    if (status != DDS::RETCODE_OK) {
        std::cerr << "ERROR: Can not send message" << std::endl;
        exit(1);
    }
}


void AckDataSenderImpl::on_requested_deadline_missed (
        DDS::DataReader_ptr,
        const DDS::RequestedDeadlineMissedStatus &)
{
}

void AckDataSenderImpl::on_requested_incompatible_qos (
        DDS::DataReader_ptr,
        const DDS::RequestedIncompatibleQosStatus &)
{
}

void AckDataSenderImpl::on_liveliness_changed (
        DDS::DataReader_ptr,
        const DDS::LivelinessChangedStatus &)
{
}

void AckDataSenderImpl::on_subscription_matched (
        DDS::DataReader_ptr,
        const DDS::SubscriptionMatchedStatus &)
{
}

void AckDataSenderImpl::on_sample_rejected(
        DDS::DataReader_ptr,
        const DDS::SampleRejectedStatus&)
{
}

void AckDataSenderImpl::on_sample_lost(
        DDS::DataReader_ptr,
        const DDS::SampleLostStatus&)
{
}