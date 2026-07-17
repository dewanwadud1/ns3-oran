/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
#include "oran-command-nr-2-nr-handover.h"

#include "ns3/log.h"
#include "ns3/uinteger.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranCommandNr2NrHandover");

NS_OBJECT_ENSURE_REGISTERED(OranCommandNr2NrHandover);

TypeId
OranCommandNr2NrHandover::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::OranCommandNr2NrHandover")
            .SetParent<OranCommand>()
            .AddConstructor<OranCommandNr2NrHandover>()
            .AddAttribute("TargetCellId",
                          "The ID of the NR cell to handover to.",
                          UintegerValue(0),
                          MakeUintegerAccessor(&OranCommandNr2NrHandover::m_targetCellId),
                          MakeUintegerChecker<uint16_t>())
            .AddAttribute("TargetRnti",
                          "The current RNTI of the UE to handover.",
                          UintegerValue(0),
                          MakeUintegerAccessor(&OranCommandNr2NrHandover::m_targetRnti),
                          MakeUintegerChecker<uint16_t>());

    return tid;
}

OranCommandNr2NrHandover::OranCommandNr2NrHandover()
    : OranCommand()
{
    NS_LOG_FUNCTION(this);
}

OranCommandNr2NrHandover::~OranCommandNr2NrHandover()
{
    NS_LOG_FUNCTION(this);
}

std::string
OranCommandNr2NrHandover::ToString() const
{
    NS_LOG_FUNCTION(this);

    std::stringstream ss;

    ss << "OranCommandNr2NrHandover("
       << "TargetE2NodeId = " << GetTargetE2NodeId() << "; TargetCellId = " << m_targetCellId
       << "; TargetRnti = " << m_targetRnti << ")";

    return ss.str();
}

uint16_t
OranCommandNr2NrHandover::GetTargetCellId() const
{
    NS_LOG_FUNCTION(this);

    return m_targetCellId;
}

uint16_t
OranCommandNr2NrHandover::GetTargetRnti() const
{
    NS_LOG_FUNCTION(this);

    return m_targetRnti;
}

} // namespace ns3
