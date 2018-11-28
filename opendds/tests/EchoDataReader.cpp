// -*- C++ -*-
//
#include "EchoDataReader.h"
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

EchoDataReaderImpl::EchoDataReaderImpl()
        :reader_ (),
         dr_servant_ (),
         use_zero_copy_(false)
{

}

EchoDataReaderImpl::~EchoDataReaderImpl ()
{
}

void EchoDataReaderImpl::init(DDS::DataReader_ptr dr,
                                     bool use_zero_copy_read)
{
    this->reader_ = DDS::DataReader::_duplicate (dr);
    use_zero_copy_ = use_zero_copy_read;

    this->dr_servant_ =
            PubMessageDataReader::_narrow(this->reader_.in());
}


void EchoDataReaderImpl::on_data_available(DDS::DataReader_ptr)
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


        if (onDataCallback != NULL)
            onDataCallback(&messageZC[0]);

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

        if (onDataCallback != NULL)
            onDataCallback(&message);
    }

    if (status == DDS::RETCODE_OK) {
//      cout << "PubMessage: seqnum    = " << message.seqnum << endl;
    } else if (status == DDS::RETCODE_NO_DATA) {
        cerr << "ERROR: reader received DDS::RETCODE_NO_DATA!" << endl;
    } else {
        cerr << "ERROR: read Message: Error: " <<  status << endl;
    }
}

void EchoDataReaderImpl::on_requested_deadline_missed (
        DDS::DataReader_ptr,
        const DDS::RequestedDeadlineMissedStatus &)
{
}

void EchoDataReaderImpl::on_requested_incompatible_qos (
        DDS::DataReader_ptr,
        const DDS::RequestedIncompatibleQosStatus &)
{
}

void EchoDataReaderImpl::on_liveliness_changed (
        DDS::DataReader_ptr,
        const DDS::LivelinessChangedStatus &)
{
}

void EchoDataReaderImpl::on_subscription_matched (
        DDS::DataReader_ptr,
        const DDS::SubscriptionMatchedStatus &)
{
}

void EchoDataReaderImpl::on_sample_rejected(
        DDS::DataReader_ptr,
        const DDS::SampleRejectedStatus&)
{
}

void EchoDataReaderImpl::on_sample_lost(
        DDS::DataReader_ptr,
        const DDS::SampleLostStatus&)
{
}