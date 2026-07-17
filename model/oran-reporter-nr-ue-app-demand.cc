/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
#include "oran-reporter-nr-ue-app-demand.h"

#include "oran-report-nr-ue-app-demand.h"

#include "ns3/abort.h"
#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/uinteger.h"

#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranReporterNrUeAppDemand");
NS_OBJECT_ENSURE_REGISTERED(OranReporterNrUeAppDemand);

TypeId
OranReporterNrUeAppDemand::GetTypeId()
{
    static TypeId tid = TypeId("ns3::OranReporterNrUeAppDemand")
                            .SetParent<OranReporter>()
                            .AddConstructor<OranReporterNrUeAppDemand>();

    return tid;
}

OranReporterNrUeAppDemand::OranReporterNrUeAppDemand()
{
    NS_LOG_FUNCTION(this);
}

OranReporterNrUeAppDemand::~OranReporterNrUeAppDemand()
{
    NS_LOG_FUNCTION(this);
}

void
OranReporterNrUeAppDemand::ReportDemand(double demandMbps)
{
    NS_LOG_FUNCTION(this << demandMbps);

    if (!std::isfinite(demandMbps))
    {
        return;
    }

    if (m_active)
    {
        NS_ABORT_MSG_IF(m_terminator == nullptr,
                        "Attempting to generate reports in reporter with NULL E2 Terminator");

        Ptr<OranReportNrUeAppDemand> report = CreateObject<OranReportNrUeAppDemand>();
        report->SetAttribute("ReporterE2NodeId", UintegerValue(m_terminator->GetE2NodeId()));
        report->SetAttribute("Time", TimeValue(Simulator::Now()));
        report->SetAttribute("DemandMbps", DoubleValue(demandMbps));

        m_reports.push_back(report);
    }
}

std::vector<Ptr<OranReport>>
OranReporterNrUeAppDemand::GenerateReports()
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
