/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
#include "oran-command-lte-2-lte-cell-parameter.h"

#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/string.h"

#include <sstream>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranCommandLte2LteCellParameter");
NS_OBJECT_ENSURE_REGISTERED(OranCommandLte2LteCellParameter);

TypeId
OranCommandLte2LteCellParameter::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::OranCommandLte2LteCellParameter")
            .SetParent<OranCommand>()
            .AddConstructor<OranCommandLte2LteCellParameter>()
            .AddAttribute("ParameterName",
                          "LTE cell-control parameter name: CIO, TTT, HYS, or RET.",
                          StringValue("CIO"),
                          MakeStringAccessor(&OranCommandLte2LteCellParameter::m_parameterName),
                          MakeStringChecker())
            .AddAttribute("Value",
                          "Parameter value or delta.",
                          DoubleValue(0.0),
                          MakeDoubleAccessor(&OranCommandLte2LteCellParameter::m_value),
                          MakeDoubleChecker<double>())
            .AddAttribute("IsDelta",
                          "Whether Value is a delta to add to the current parameter.",
                          BooleanValue(false),
                          MakeBooleanAccessor(&OranCommandLte2LteCellParameter::m_isDelta),
                          MakeBooleanChecker());
    return tid;
}

OranCommandLte2LteCellParameter::OranCommandLte2LteCellParameter()
{
    NS_LOG_FUNCTION(this);
}

OranCommandLte2LteCellParameter::~OranCommandLte2LteCellParameter()
{
    NS_LOG_FUNCTION(this);
}

std::string
OranCommandLte2LteCellParameter::ToString() const
{
    std::ostringstream ss;
    ss << "OranCommandLte2LteCellParameter("
       << "TargetE2NodeId=" << GetTargetE2NodeId() << ";ParameterName=" << m_parameterName
       << ";Value=" << m_value << ";IsDelta=" << m_isDelta << ")";
    return ss.str();
}

std::string
OranCommandLte2LteCellParameter::GetParameterName() const
{
    return m_parameterName;
}

double
OranCommandLte2LteCellParameter::GetValue() const
{
    return m_value;
}

bool
OranCommandLte2LteCellParameter::IsDelta() const
{
    return m_isDelta;
}

} // namespace ns3
