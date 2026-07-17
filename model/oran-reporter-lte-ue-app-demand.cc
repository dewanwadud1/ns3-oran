/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
#include "oran-reporter-lte-ue-app-demand.h"

#include "oran-report-lte-ue-app-demand.h"

#include "ns3/abort.h"
#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/uinteger.h"

#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranReporterLteUeAppDemand");
NS_OBJECT_ENSURE_REGISTERED(OranReporterLteUeAppDemand);

TypeId
OranReporterLteUeAppDemand::GetTypeId()
{
    static TypeId tid = TypeId("ns3::OranReporterLteUeAppDemand")
                            .SetParent<OranReporter>()
                            .AddConstructor<OranReporterLteUeAppDemand>();

    return tid;
}

OranReporterLteUeAppDemand::OranReporterLteUeAppDemand()
{
    NS_LOG_FUNCTION(this);
}

OranReporterLteUeAppDemand::~OranReporterLteUeAppDemand()
{
    NS_LOG_FUNCTION(this);
}

void
OranReporterLteUeAppDemand::ReportDemand(double demandMbps)
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

        Ptr<OranReportLteUeAppDemand> report = CreateObject<OranReportLteUeAppDemand>();
        report->SetAttribute("ReporterE2NodeId", UintegerValue(m_terminator->GetE2NodeId()));
        report->SetAttribute("Time", TimeValue(Simulator::Now()));
        report->SetAttribute("DemandMbps", DoubleValue(demandMbps));

        m_reports.push_back(report);
    }
}

std::vector<Ptr<OranReport>>
OranReporterLteUeAppDemand::GenerateReports()
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
