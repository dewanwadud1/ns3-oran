/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
#include "oran-command-nr-2-nr-cell-parameter.h"

#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/string.h"

#include <sstream>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranCommandNr2NrCellParameter");
NS_OBJECT_ENSURE_REGISTERED(OranCommandNr2NrCellParameter);

TypeId
OranCommandNr2NrCellParameter::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::OranCommandNr2NrCellParameter")
            .SetParent<OranCommand>()
            .AddConstructor<OranCommandNr2NrCellParameter>()
            .AddAttribute("ParameterName",
                          "NR cell-control parameter name: CIO, TTT, HYS, or RET.",
                          StringValue("CIO"),
                          MakeStringAccessor(&OranCommandNr2NrCellParameter::m_parameterName),
                          MakeStringChecker())
            .AddAttribute("Value",
                          "Parameter value or delta.",
                          DoubleValue(0.0),
                          MakeDoubleAccessor(&OranCommandNr2NrCellParameter::m_value),
                          MakeDoubleChecker<double>())
            .AddAttribute("IsDelta",
                          "Whether Value is a delta to add to the current parameter.",
                          BooleanValue(false),
                          MakeBooleanAccessor(&OranCommandNr2NrCellParameter::m_isDelta),
                          MakeBooleanChecker());
    return tid;
}

OranCommandNr2NrCellParameter::OranCommandNr2NrCellParameter()
{
    NS_LOG_FUNCTION(this);
}

OranCommandNr2NrCellParameter::~OranCommandNr2NrCellParameter()
{
    NS_LOG_FUNCTION(this);
}

std::string
OranCommandNr2NrCellParameter::ToString() const
{
    std::ostringstream ss;
    ss << "OranCommandNr2NrCellParameter("
       << "TargetE2NodeId=" << GetTargetE2NodeId() << ";ParameterName=" << m_parameterName
       << ";Value=" << m_value << ";IsDelta=" << m_isDelta << ")";
    return ss.str();
}

std::string
OranCommandNr2NrCellParameter::GetParameterName() const
{
    return m_parameterName;
}

double
OranCommandNr2NrCellParameter::GetValue() const
{
    return m_value;
}

bool
OranCommandNr2NrCellParameter::IsDelta() const
{
    return m_isDelta;
}

} // namespace ns3
