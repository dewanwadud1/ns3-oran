/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
#include "oran-report-nr-ue-app-demand.h"

#include "ns3/double.h"
#include "ns3/log.h"

#include <sstream>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranReportNrUeAppDemand");
NS_OBJECT_ENSURE_REGISTERED(OranReportNrUeAppDemand);

TypeId
OranReportNrUeAppDemand::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::OranReportNrUeAppDemand")
            .SetParent<OranReport>()
            .AddConstructor<OranReportNrUeAppDemand>()
            .AddAttribute("DemandMbps",
                          "Observed application-layer demand (Mbps) over the last interval.",
                          DoubleValue(),
                          MakeDoubleAccessor(&OranReportNrUeAppDemand::m_demandMbps),
                          MakeDoubleChecker<double>());

    return tid;
}

OranReportNrUeAppDemand::OranReportNrUeAppDemand()
{
    NS_LOG_FUNCTION(this);
}

OranReportNrUeAppDemand::~OranReportNrUeAppDemand()
{
    NS_LOG_FUNCTION(this);
}

std::string
OranReportNrUeAppDemand::ToString() const
{
    NS_LOG_FUNCTION(this);

    std::stringstream ss;
    Time time = GetTime();

    ss << "OranReportNrUeAppDemand("
       << "E2NodeId=" << GetReporterE2NodeId() << ";Time=" << time.As(Time::S)
       << ";DemandMbps=" << m_demandMbps << ")";

    return ss.str();
}

double
OranReportNrUeAppDemand::GetDemandMbps() const
{
    NS_LOG_FUNCTION(this);

    return m_demandMbps;
}

} // namespace ns3
