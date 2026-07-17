/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
#include "oran-reporter-nr-ue-rsrp-rsrq.h"

#include "oran-report-nr-ue-rsrp-rsrq.h"

#include "ns3/abort.h"
#include "ns3/address.h"
#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/packet.h"
#include "ns3/simulator.h"
#include "ns3/uinteger.h"

#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranReporterNrUeRsrpRsrq");
NS_OBJECT_ENSURE_REGISTERED(OranReporterNrUeRsrpRsrq);

TypeId
OranReporterNrUeRsrpRsrq::GetTypeId()
{
    static TypeId tid = TypeId("ns3::OranReporterNrUeRsrpRsrq")
                            .SetParent<OranReporter>()
                            .AddConstructor<OranReporterNrUeRsrpRsrq>();

    return tid;
}

OranReporterNrUeRsrpRsrq::OranReporterNrUeRsrpRsrq()
{
    NS_LOG_FUNCTION(this);
}

OranReporterNrUeRsrpRsrq::~OranReporterNrUeRsrpRsrq()
{
    NS_LOG_FUNCTION(this);
}

void
OranReporterNrUeRsrpRsrq::ReportRsrpRsrq(uint16_t rnti,
                                         uint16_t cellId,
                                         double rsrp,
                                         double rsrq,
                                         bool isServingCell,
                                         uint8_t componentCarrierId)
{
    NS_LOG_FUNCTION(this << +rnti << +cellId << rsrp << rsrq << isServingCell
                         << componentCarrierId);

    // Mirrors the LTE reporter's guard: skip non-finite RSRP/RSRQ samples
    // (e.g. around a radio link failure) instead of aborting the simulation.
    if (!std::isfinite(rsrp) || !std::isfinite(rsrq))
    {
        return;
    }

    if (m_active)
    {
        NS_ABORT_MSG_IF(m_terminator == nullptr,
                        "Attempting to generate reports in reporter with NULL E2 Terminator");

        Ptr<OranReportNrUeRsrpRsrq> report = CreateObject<OranReportNrUeRsrpRsrq>();
        report->SetAttribute("ReporterE2NodeId", UintegerValue(m_terminator->GetE2NodeId()));
        report->SetAttribute("Time", TimeValue(Simulator::Now()));
        report->SetAttribute("Rnti", UintegerValue(rnti));
        report->SetAttribute("CellId", UintegerValue(cellId));
        report->SetAttribute("Rsrp", DoubleValue(rsrp));
        report->SetAttribute("Rsrq", DoubleValue(rsrq));
        report->SetAttribute("IsServingCell", BooleanValue(isServingCell));
        report->SetAttribute("ComponentCarrierId", UintegerValue(componentCarrierId));

        m_reports.push_back(report);
    }
}

std::vector<Ptr<OranReport>>
OranReporterNrUeRsrpRsrq::GenerateReports()
{
    NS_LOG_FUNCTION(this);

    std::vector<Ptr<OranReport>> reports;

    if (m_active)
    {
        reports = m_reports;
        m_reports.clear();
    }

    return reports;
}

} // namespace ns3
