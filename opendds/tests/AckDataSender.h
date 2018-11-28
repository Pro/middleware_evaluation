// -*- C++ -*-
//
#ifndef ACKDATASENDER_LISTENER_IMPL
#define ACKDATASENDER_LISTENER_IMPL

#include <dds/DdsDcpsSubscriptionC.h>
#include <dds/DdsDcpsPublicationC.h>
#include "ace/High_Res_Timer.h"
#include "DDSPerfTestTypeSupportImpl.h"

#if !defined (ACE_LACKS_PRAGMA_ONCE)
#pragma once
#endif /* ACE_LACKS_PRAGMA_ONCE */

class AckDataSenderImpl
        : public virtual OpenDDS::DCPS::LocalObject<DDS::DataReaderListener>
{
public:
    AckDataSenderImpl ();

    void init (DDS::DataReader_ptr dr, DDS::DataWriter_ptr dw, bool use_zero_copy_read);

    virtual ~AckDataSenderImpl (void);

    virtual void on_requested_deadline_missed (
            DDS::DataReader_ptr reader,
            const DDS::RequestedDeadlineMissedStatus & status);

    virtual void on_requested_incompatible_qos (
            DDS::DataReader_ptr reader,
            const DDS::RequestedIncompatibleQosStatus & status);

    virtual void on_liveliness_changed (
            DDS::DataReader_ptr reader,
            const DDS::LivelinessChangedStatus & status);

    virtual void on_subscription_matched (
            DDS::DataReader_ptr reader,
            const DDS::SubscriptionMatchedStatus & status);

    virtual void on_sample_rejected(
            DDS::DataReader_ptr reader,
            const DDS::SampleRejectedStatus& status);

    virtual void on_data_available(
            DDS::DataReader_ptr reader);

    virtual void on_sample_lost(
            DDS::DataReader_ptr reader,
            const DDS::SampleLostStatus& status);

    void sendMessage(DDSPerfTest::PubMessage &pubmessage_data);

    bool isAckReceived() {
        return ackRecieved;
    }
private:

    DDS::DataWriter_var writer_;
    DDS::DataReader_var reader_;
    DDSPerfTest::AckMessageDataReader_var dr_servant_;
    DDSPerfTest::PubMessageDataWriter_var dw_servant_;
    DDS::InstanceHandle_t handle_;
    bool  use_zero_copy_;

    bool ackRecieved;
};

#endif /* ACKDATASENDER_LISTENER_IMPL  */
