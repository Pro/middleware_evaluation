// -*- C++ -*-
//
#include "EchoDataSender.h"
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

EchoDataSenderImpl::EchoDataSenderImpl()
        :writer_ (),
         reader_ (),
         dr_servant_ (),
         dw_servant_ (),
         handle_ (),
         use_zero_copy_(false)
{

}

EchoDataSenderImpl::~EchoDataSenderImpl ()
{
}

void EchoDataSenderImpl::init(DDS::DataReader_ptr dr,
                                     DDS::DataWriter_ptr dw,
                                     bool use_zero_copy_read)
{
    this->writer_ = DDS::DataWriter::_duplicate (dw);
    this->reader_ = DDS::DataReader::_duplicate (dr);
    use_zero_copy_ = use_zero_copy_read;

    this->dr_servant_ =
            PubMessageDataReader::_narrow(this->reader_.in());

    this->dw_servant_ =
            PubMessageDataWriter::_narrow (this->writer_.in ());
    DDSPerfTest::PubMessage msg;
    this->handle_ = this->dw_servant_->register_instance(msg);
}


void EchoDataSenderImpl::on_data_available(DDS::DataReader_ptr)
{
    CORBA::Long rec_size;
    DDS::ReturnCode_t status;

    if (use_zero_copy_)
    {
        ::CORBA::Long max_read_samples = 1;
        PubMessageSeq messageZC;
        DDS::SampleInfoSeq siZC;
        status = this->dr_servant_->take(messageZC,
                                         siZC,
                                         max_read_samples,
                                         ::DDS::NOT_READ_SAMPLE_STATE,
                                         ::DDS::ANY_VIEW_STATE,
                                         ::DDS::ANY_INSTANCE_STATE);

        if (status != DDS::RETCODE_OK) {
            std::cerr << "ERROR: Can not take message from ECHO" << std::endl;
            exit(1);
        }

        rec_size = messageZC[0].buffer.length();
        status = this->dr_servant_->return_loan(messageZC, siZC);

        if (status != DDS::RETCODE_OK) {
            std::cerr << "ERROR: Can not return loan" << std::endl;
            exit(1);
        }
    }
    else
    {
        static DDSPerfTest::PubMessage message;
        static DDS::SampleInfo si;
        status = this->dr_servant_->take_next_sample(message, si) ;

        rec_size = message.buffer.length();
    }

    if (status == DDS::RETCODE_OK) {
//      cout << "PubMessage: seqnum    = " << message.seqnum << endl;
    } else if (status == DDS::RETCODE_NO_DATA) {
        cerr << "ERROR: reader received DDS::RETCODE_NO_DATA!" << endl;
    } else {
        cerr << "ERROR: read Message: Error: " <<  status << endl;
    }

    echoRecieved = true;

}


void EchoDataSenderImpl::sendMessage(DDSPerfTest::PubMessage &pubmessage_data) {
    echoRecieved = false;
    DDS::ReturnCode_t status = this->dw_servant_->write (pubmessage_data, this->handle_);
    if (status != DDS::RETCODE_OK) {
        std::cerr << "ERROR: Can not send message" << std::endl;
        exit(1);
    }
}


void EchoDataSenderImpl::on_requested_deadline_missed (
        DDS::DataReader_ptr,
        const DDS::RequestedDeadlineMissedStatus &)
{
}

void EchoDataSenderImpl::on_requested_incompatible_qos (
        DDS::DataReader_ptr,
        const DDS::RequestedIncompatibleQosStatus &)
{
}

void EchoDataSenderImpl::on_liveliness_changed (
        DDS::DataReader_ptr,
        const DDS::LivelinessChangedStatus &)
{
}

void EchoDataSenderImpl::on_subscription_matched (
        DDS::DataReader_ptr,
        const DDS::SubscriptionMatchedStatus &)
{
}

void EchoDataSenderImpl::on_sample_rejected(
        DDS::DataReader_ptr,
        const DDS::SampleRejectedStatus&)
{
}

void EchoDataSenderImpl::on_sample_lost(
        DDS::DataReader_ptr,
        const DDS::SampleLostStatus&)
{
}