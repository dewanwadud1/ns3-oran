/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
/**
 * \ingroup oran
 *
 * Implementation of the NR energy-efficiency report.
 */

#include "oran-report-nr-energy-efficiency.h"

#include "oran-report.h"
#include <ns3/log.h>
#include <ns3/double.h>
#include <sstream>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("OranReportNrEnergyEfficiency");
NS_OBJECT_ENSURE_REGISTERED(OranReportNrEnergyEfficiency);

TypeId
OranReportNrEnergyEfficiency::GetTypeId()
{
  static TypeId tid = TypeId("ns3::OranReportNrEnergyEfficiency")
      .SetParent<OranReport>()
      .AddConstructor<OranReportNrEnergyEfficiency>()
      .AddAttribute(
          "EnergyRemaining",
          "The remaining joules",
          DoubleValue(),
          MakeDoubleAccessor(&OranReportNrEnergyEfficiency::m_energyRemaining),
          MakeDoubleChecker<double>());
  return tid;
}

OranReportNrEnergyEfficiency::OranReportNrEnergyEfficiency()
{
  NS_LOG_FUNCTION(this);
}

OranReportNrEnergyEfficiency::~OranReportNrEnergyEfficiency()
{
  NS_LOG_FUNCTION(this);
}

std::string
OranReportNrEnergyEfficiency::ToString() const
{
  NS_LOG_FUNCTION(this);
  std::stringstream ss;
  Time time = GetTime();
  ss << "OranReportNrEnergyEfficiency(";
  ss << "E2NodeId=" << GetReporterE2NodeId();
  ss << ";Time=" << time.As(Time::S);
  ss << ";EnergyRemaining=" << m_energyRemaining;
  ss << ")";
  return ss.str();
}

double
OranReportNrEnergyEfficiency::GetNrEnergyRemaining() const
{
  NS_LOG_FUNCTION(this);
  return m_energyRemaining;
}

} // namespace ns3
