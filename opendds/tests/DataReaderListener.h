// -*- C++ -*-
//
#ifndef DATAREADER_LISTENER_IMPL
#define DATAREADER_LISTENER_IMPL

#include <dds/DdsDcpsSubscriptionExtC.h>
#include <dds/DCPS/LocalObject.h>
#include <iostream>

#if !defined (ACE_LACKS_PRAGMA_ONCE)
#pragma once
#endif /* ACE_LACKS_PRAGMA_ONCE */

using namespace std;

class DataReaderListenerImpl
        : public virtual OpenDDS::DCPS::LocalObject<OpenDDS::DCPS::DataReaderListener>
{
public:
    DataReaderListenerImpl (): num_reads_(0), onDataCallback(NULL) {};

    virtual ~DataReaderListenerImpl () {};

    virtual void on_requested_deadline_missed (
            DDS::DataReader_ptr reader,
            const DDS::RequestedDeadlineMissedStatus & status)
    {
        cerr << "DataReaderListenerImpl::on_requested_deadline_missed" << endl;
    }

    virtual void on_requested_incompatible_qos (
            DDS::DataReader_ptr reader,
            const DDS::RequestedIncompatibleQosStatus & status)
    {
        cerr << "DataReaderListenerImpl::on_requested_incompatible_qos" << endl;
    }

    virtual void on_liveliness_changed (
            DDS::DataReader_ptr reader,
            const DDS::LivelinessChangedStatus & status)
    {
        //cerr << "DataReaderListenerImpl::on_liveliness_changed" << endl;
    }


    virtual void on_subscription_matched (
            DDS::DataReader_ptr reader,
            const DDS::SubscriptionMatchedStatus & status)
    {
        // cerr << "DataReaderListenerImpl::on_subscription_matched" << endl;
    }

    virtual void on_sample_rejected(
            DDS::DataReader_ptr reader,
            const DDS::SampleRejectedStatus& status)
    {
        cerr << "DataReaderListenerImpl::on_sample_rejected" << endl;
    }

    virtual void on_data_available(
            DDS::DataReader_ptr reader)
    {
        ++num_reads_;

        if (onDataCallback != NULL)
            onDataCallback(reader);

    }

    virtual void on_sample_lost(
            DDS::DataReader_ptr reader,
            const DDS::SampleLostStatus& status)
    {
        cerr << "DataReaderListenerImpl::on_sample_lost" << endl;
    }

    virtual void on_subscription_disconnected (
            DDS::DataReader_ptr reader,
            const ::OpenDDS::DCPS::SubscriptionDisconnectedStatus & status)
    {
        cerr << "DataReaderListenerImpl::on_subscription_disconnected" << endl;
    }

    virtual void on_subscription_reconnected (
            DDS::DataReader_ptr reader,
            const ::OpenDDS::DCPS::SubscriptionReconnectedStatus & status)
    {
        cerr << "DataReaderListenerImpl::on_subscription_reconnected" << endl;
    }

    virtual void on_subscription_lost (
            DDS::DataReader_ptr reader,
            const ::OpenDDS::DCPS::SubscriptionLostStatus & status)
    {
        cerr << "DataReaderListenerImpl::on_subscription_lost" << endl;
    }

    virtual void on_budget_exceeded(
            DDS::DataReader_ptr reader,
            const ::OpenDDS::DCPS::BudgetExceededStatus& status)
    {
        cerr << "DataReaderListenerImpl::on_budget_exceeded" << endl;
    }

    long num_reads() const {
        return num_reads_;
    }

    void setDataCallback(void (*callback)(DDS::DataReader_ptr reader)) {
        this->onDataCallback = callback;
    }

private:

    DDS::DataReader_var reader_;
    long                  num_reads_;

    void (*onDataCallback)(DDS::DataReader_ptr reader);
};

#endif /* DATAREADER_LISTENER_IMPL  */
