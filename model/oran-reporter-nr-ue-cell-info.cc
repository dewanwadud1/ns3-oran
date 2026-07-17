/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
#include "oran-reporter-nr-ue-cell-info.h"

#include "oran-report-nr-ue-cell-info.h"

#include "ns3/abort.h"
#include "ns3/log.h"
#include "ns3/nr-ue-net-device.h"
#include "ns3/nr-ue-rrc.h"
#include "ns3/mobility-model.h"
#include "ns3/simulator.h"
#include "ns3/uinteger.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranReporterNrUeCellInfo");

NS_OBJECT_ENSURE_REGISTERED(OranReporterNrUeCellInfo);

TypeId
OranReporterNrUeCellInfo::GetTypeId()
{
    static TypeId tid = TypeId("ns3::OranReporterNrUeCellInfo")
                            .SetParent<OranReporter>()
                            .AddConstructor<OranReporterNrUeCellInfo>();

    return tid;
}

OranReporterNrUeCellInfo::OranReporterNrUeCellInfo()
    : OranReporter()
{
    NS_LOG_FUNCTION(this);
}

OranReporterNrUeCellInfo::~OranReporterNrUeCellInfo()
{
    NS_LOG_FUNCTION(this);
}

std::vector<Ptr<OranReport>>
OranReporterNrUeCellInfo::GenerateReports()
{
    NS_LOG_FUNCTION(this);

    std::vector<Ptr<OranReport>> reports;
    if (m_active)
    {
        NS_ABORT_MSG_IF(m_terminator == nullptr,
                        "Attempting to generate reports in reporter with NULL E2 Terminator");

        Ptr<NrUeNetDevice> nrUeNetDev = nullptr;
        Ptr<Node> node = m_terminator->GetNode();
        Ptr<OranReportNrUeCellInfo> cellInfoReport = CreateObject<OranReportNrUeCellInfo>();

        for (uint32_t idx = 0; nrUeNetDev == nullptr && idx < node->GetNDevices(); idx++)
        {
            nrUeNetDev = node->GetDevice(idx)->GetObject<NrUeNetDevice>();
        }

        NS_ABORT_MSG_IF(nrUeNetDev == nullptr, "Unable to find appropriate network device");

        Ptr<NrUeRrc> nrUeRrc = nrUeNetDev->GetRrc();

        cellInfoReport->SetAttribute("ReporterE2NodeId",
                                     UintegerValue(m_terminator->GetE2NodeId()));
        cellInfoReport->SetAttribute("CellId", UintegerValue(nrUeRrc->GetCellId()));
        cellInfoReport->SetAttribute("Rnti", UintegerValue(nrUeRrc->GetRnti()));
        cellInfoReport->SetAttribute("Time", TimeValue(Simulator::Now()));

        reports.push_back(cellInfoReport);
    }

    return reports;
}

} // namespace ns3
