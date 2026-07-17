/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
#include "oran-report-lte-ue-app-demand.h"

#include "ns3/double.h"
#include "ns3/log.h"

#include <sstream>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranReportLteUeAppDemand");
NS_OBJECT_ENSURE_REGISTERED(OranReportLteUeAppDemand);

TypeId
OranReportLteUeAppDemand::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::OranReportLteUeAppDemand")
            .SetParent<OranReport>()
            .AddConstructor<OranReportLteUeAppDemand>()
            .AddAttribute("DemandMbps",
                          "Observed application-layer demand (Mbps) over the last interval.",
                          DoubleValue(),
                          MakeDoubleAccessor(&OranReportLteUeAppDemand::m_demandMbps),
                          MakeDoubleChecker<double>());

    return tid;
}

OranReportLteUeAppDemand::OranReportLteUeAppDemand()
{
    NS_LOG_FUNCTION(this);
}

OranReportLteUeAppDemand::~OranReportLteUeAppDemand()
{
    NS_LOG_FUNCTION(this);
}

std::string
OranReportLteUeAppDemand::ToString() const
{
    NS_LOG_FUNCTION(this);

    std::stringstream ss;
    Time time = GetTime();

    ss << "OranReportLteUeAppDemand("
       << "E2NodeId=" << GetReporterE2NodeId() << ";Time=" << time.As(Time::S)
       << ";DemandMbps=" << m_demandMbps << ")";

    return ss.str();
}

double
OranReportLteUeAppDemand::GetDemandMbps() const
{
    NS_LOG_FUNCTION(this);

    return m_demandMbps;
}

} // namespace ns3
