/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
#include "oran-report-nr-ue-rsrp-rsrq.h"

#include "oran-report.h"

#include "ns3/abort.h"
#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/uinteger.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranReportNrUeRsrpRsrq");
NS_OBJECT_ENSURE_REGISTERED(OranReportNrUeRsrpRsrq);

TypeId
OranReportNrUeRsrpRsrq::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::OranReportNrUeRsrpRsrq")
            .SetParent<OranReport>()
            .AddConstructor<OranReportNrUeRsrpRsrq>()
            .AddAttribute("Rnti",
                          "The RNTI.",
                          UintegerValue(),
                          MakeUintegerAccessor(&OranReportNrUeRsrpRsrq::m_rnti),
                          MakeUintegerChecker<uint16_t>())
            .AddAttribute("CellId",
                          "The cell ID.",
                          UintegerValue(),
                          MakeUintegerAccessor(&OranReportNrUeRsrpRsrq::m_cellId),
                          MakeUintegerChecker<uint16_t>())
            .AddAttribute("Rsrp",
                          "The RSRP.",
                          DoubleValue(),
                          MakeDoubleAccessor(&OranReportNrUeRsrpRsrq::m_rsrp),
                          MakeDoubleChecker<double>())
            .AddAttribute("Rsrq",
                          "The RSRQ.",
                          DoubleValue(),
                          MakeDoubleAccessor(&OranReportNrUeRsrpRsrq::m_rsrq),
                          MakeDoubleChecker<double>())
            .AddAttribute("IsServingCell",
                          "The flag that indicates if this for the serving cell.",
                          BooleanValue(),
                          MakeBooleanAccessor(&OranReportNrUeRsrpRsrq::m_isServingCell),
                          MakeBooleanChecker())
            .AddAttribute("ComponentCarrierId",
                          "The component carrier ID.",
                          UintegerValue(),
                          MakeUintegerAccessor(&OranReportNrUeRsrpRsrq::m_componentCarrierId),
                          MakeUintegerChecker<uint16_t>());

    return tid;
}

OranReportNrUeRsrpRsrq::OranReportNrUeRsrpRsrq()
{
    NS_LOG_FUNCTION(this);
}

OranReportNrUeRsrpRsrq::~OranReportNrUeRsrpRsrq()
{
    NS_LOG_FUNCTION(this);
}

std::string
OranReportNrUeRsrpRsrq::ToString() const
{
    NS_LOG_FUNCTION(this);

    std::stringstream ss;
    Time time = GetTime();

    ss << "OranReportNrUeRsrpRsrq("
       << "E2NodeId=" << GetReporterE2NodeId() << ";Time=" << time.As(Time::S)
       << ";RNTI=" << +m_rnti << ";Cell ID=" << +m_cellId << ";RSRP=" << m_rsrp
       << ";RSRQ=" << m_rsrq << ";Is Serving Cell=" << m_isServingCell
       << ";Component Carrier ID=" << +m_componentCarrierId << ")";

    return ss.str();
}

uint16_t
OranReportNrUeRsrpRsrq::GetRnti() const
{
    NS_LOG_FUNCTION(this);

    return m_rnti;
}

uint16_t
OranReportNrUeRsrpRsrq::GetCellId() const
{
    NS_LOG_FUNCTION(this);

    return m_cellId;
}

double
OranReportNrUeRsrpRsrq::GetRsrp() const
{
    NS_LOG_FUNCTION(this);

    return m_rsrp;
}

double
OranReportNrUeRsrpRsrq::GetRsrq() const
{
    NS_LOG_FUNCTION(this);

    return m_rsrq;
}

bool
OranReportNrUeRsrpRsrq::GetIsServingCell() const
{
    NS_LOG_FUNCTION(this);

    return m_isServingCell;
}

uint16_t
OranReportNrUeRsrpRsrq::GetComponentCarrierId() const
{
    NS_LOG_FUNCTION(this);

    return m_componentCarrierId;
}

} // namespace ns3
