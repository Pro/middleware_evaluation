// -*- C++ -*-
//
#ifndef EchoDataReader_LISTENER_IMPL
#define EchoDataReader_LISTENER_IMPL

#include <dds/DdsDcpsSubscriptionC.h>
#include <dds/DdsDcpsPublicationC.h>
#include "ace/High_Res_Timer.h"
#include "DDSPerfTestTypeSupportImpl.h"

#if !defined (ACE_LACKS_PRAGMA_ONCE)
#pragma once
#endif /* ACE_LACKS_PRAGMA_ONCE */

class EchoDataReaderImpl
        : public virtual OpenDDS::DCPS::LocalObject<DDS::DataReaderListener>
{
public:
    EchoDataReaderImpl ();

    void init (DDS::DataReader_ptr dr, bool use_zero_copy_read);

    virtual ~EchoDataReaderImpl (void);

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

    void setDataCallback(void (*callback)(DDSPerfTest::PubMessage *msg)) {
        this->onDataCallback = callback;
    }
private:

    DDS::DataReader_var reader_;
    DDSPerfTest::PubMessageDataReader_var dr_servant_;
    bool  use_zero_copy_;

    void (*onDataCallback)(DDSPerfTest::PubMessage *msg);
};

#endif /* EchoDataReader_LISTENER_IMPL  */
