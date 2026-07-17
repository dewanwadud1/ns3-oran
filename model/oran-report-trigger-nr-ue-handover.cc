/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
#include "oran-report-trigger-nr-ue-handover.h"

#include "oran-reporter.h"

#include "ns3/log.h"
#include "ns3/nr-ue-net-device.h"
#include "ns3/nr-ue-rrc.h"
#include "ns3/nstime.h"
#include "ns3/pointer.h"
#include "ns3/random-variable-stream.h"
#include "ns3/simulator.h"
#include "ns3/string.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranReportTriggerNrUeHandover");

NS_OBJECT_ENSURE_REGISTERED(OranReportTriggerNrUeHandover);

TypeId
OranReportTriggerNrUeHandover::GetTypeId()
{
    static TypeId tid = TypeId("ns3::OranReportTriggerNrUeHandover")
                            .SetParent<OranReportTrigger>()
                            .AddConstructor<OranReportTriggerNrUeHandover>();

    return tid;
}

OranReportTriggerNrUeHandover::OranReportTriggerNrUeHandover()
    : OranReportTrigger()
{
    NS_LOG_FUNCTION(this);
}

OranReportTriggerNrUeHandover::~OranReportTriggerNrUeHandover()
{
    NS_LOG_FUNCTION(this);
}

void
OranReportTriggerNrUeHandover::Activate(Ptr<OranReporter> reporter)
{
    NS_LOG_FUNCTION(this << reporter);

    if (!m_active)
    {
        Ptr<NrUeNetDevice> nrUeNetDev = nullptr;
        Ptr<Node> node = reporter->GetTerminator()->GetNode();

        for (uint32_t idx = 0; nrUeNetDev == nullptr && idx < node->GetNDevices(); idx++)
        {
            nrUeNetDev = node->GetDevice(idx)->GetObject<NrUeNetDevice>();
        }

        NS_ABORT_MSG_IF(nrUeNetDev == nullptr, "Unable to find appropriate network device");

        nrUeNetDev->GetRrc()->TraceConnectWithoutContext(
            "HandoverEndOk",
            MakeCallback(&OranReportTriggerNrUeHandover::HandoverCompleteSink, this));

        nrUeNetDev->GetRrc()->TraceConnectWithoutContext(
            "ConnectionEstablished",
            MakeCallback(&OranReportTriggerNrUeHandover::ConnectionEstablishedSink, this));
    }

    OranReportTrigger::Activate(reporter);
}

void
OranReportTriggerNrUeHandover::Deactivate()
{
    NS_LOG_FUNCTION(this);

    if (m_active)
    {
        DisconnectSink();
    }

    OranReportTrigger::Deactivate();
}

void
OranReportTriggerNrUeHandover::DoDispose()
{
    NS_LOG_FUNCTION(this);

    if (m_active)
    {
        DisconnectSink();
    }

    OranReportTrigger::DoDispose();
}

void
OranReportTriggerNrUeHandover::HandoverCompleteSink(uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
    NS_LOG_FUNCTION(this << imsi << (uint32_t)cellId << (uint32_t)rnti);

    NS_LOG_LOGIC("Handover triggering report");

    TriggerReport();
}

void
OranReportTriggerNrUeHandover::ConnectionEstablishedSink(uint64_t imsi,
                                                         uint16_t cellId,
                                                         uint16_t rnti)
{
    NS_LOG_FUNCTION(this << imsi << (uint32_t)cellId << (uint32_t)rnti);

    NS_LOG_LOGIC("Connection established triggering report");

    TriggerReport();
}

void
OranReportTriggerNrUeHandover::DisconnectSink()
{
    NS_LOG_FUNCTION(this);

    Ptr<NrUeNetDevice> nrUeNetDev = nullptr;
    Ptr<Node> node = m_reporter->GetTerminator()->GetNode();

    for (uint32_t idx = 0; nrUeNetDev == nullptr && idx < node->GetNDevices(); idx++)
    {
        nrUeNetDev = node->GetDevice(idx)->GetObject<NrUeNetDevice>();
    }

    NS_ABORT_MSG_IF(nrUeNetDev == nullptr, "Unable to find appropriate network device");

    nrUeNetDev->GetRrc()->TraceDisconnectWithoutContext(
        "HandoverEndOk",
        MakeCallback(&OranReportTriggerNrUeHandover::HandoverCompleteSink, this));

    nrUeNetDev->GetRrc()->TraceDisconnectWithoutContext(
        "ConnectionEstablished",
        MakeCallback(&OranReportTriggerNrUeHandover::ConnectionEstablishedSink, this));
}

} // namespace ns3
