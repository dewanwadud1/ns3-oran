/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
#include "oran-report-nr-ue-cell-info.h"

#include "oran-report.h"

#include "ns3/log.h"
#include "ns3/uinteger.h"
#include "ns3/vector.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranReportNrUeCellInfo");

NS_OBJECT_ENSURE_REGISTERED(OranReportNrUeCellInfo);

TypeId
OranReportNrUeCellInfo::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::OranReportNrUeCellInfo")
            .SetParent<OranReport>()
            .AddConstructor<OranReportNrUeCellInfo>()
            .AddAttribute("CellId",
                          "The ID of the cell that the UE is currently attached to.",
                          UintegerValue(),
                          MakeUintegerAccessor(&OranReportNrUeCellInfo::m_cellId),
                          MakeUintegerChecker<uint16_t>())
            .AddAttribute("Rnti",
                          "The RNTI that was assigned to this UE by the cell.",
                          UintegerValue(),
                          MakeUintegerAccessor(&OranReportNrUeCellInfo::m_rnti),
                          MakeUintegerChecker<uint16_t>());

    return tid;
}

OranReportNrUeCellInfo::OranReportNrUeCellInfo()
    : OranReport()
{
    NS_LOG_FUNCTION(this);
}

OranReportNrUeCellInfo::~OranReportNrUeCellInfo()
{
    NS_LOG_FUNCTION(this);
}

std::string
OranReportNrUeCellInfo::ToString() const
{
    NS_LOG_FUNCTION(this);

    std::stringstream ss;
    Time time = GetTime();

    ss << "OranReportNrUeCellInfo("
       << "E2NodeId=" << GetReporterE2NodeId() << ";Time=" << time.As(Time::S)
       << ";CellId=" << (uint32_t)m_cellId << ";RNTI=" << (uint32_t)m_rnti << ")";

    return ss.str();
}

uint16_t
OranReportNrUeCellInfo::GetCellId() const
{
    NS_LOG_FUNCTION(this);

    return m_cellId;
}

uint16_t
OranReportNrUeCellInfo::GetRnti() const
{
    NS_LOG_FUNCTION(this);

    return m_rnti;
}

} // namespace ns3
